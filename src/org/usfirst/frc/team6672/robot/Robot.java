package org.usfirst.frc.team6672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	
	/** Inital Variables **/
	Spark lMotor = new Spark(1);
	Spark rMotor = new Spark(0);
	DifferentialDrive myRobot = new DifferentialDrive(lMotor, rMotor);
	Timer timer = new Timer();
	
	SpeedControllerGroup driveMotors = new SpeedControllerGroup(lMotor, rMotor);
	
	/* Joystick stuff */
	Joystick stick0 = new Joystick(0);

	/* Speed Control System */
	double speedLimitMove = 0.8;
	double speedLimitRotate = 0.7;		// Has to be negative bc the joystick inverts l/r
	
	/* Sensor Systems */
	ADXRS450_Gyro spiGyro = new ADXRS450_Gyro();
	double Kp = 0.03*.35;				//Gyro converter constant (corrected)
	
	/* PWM Stuff */
	Spark motorLift = new Spark(2);
	Spark motorGear = new Spark(3);
	
	int dSLocation = DriverStation.getInstance().getLocation();
	
	Encoder lEncoder = new Encoder(0,1);
	Encoder rEncoder = new Encoder(2,3);
	double p = 0.015, i = 0, d = 0;
	
	// DRIVE STRAIGHT P VALUE: .015
	// TURN P VALUE: .03
	
	
	PIDController lController = new PIDController(.05, 0, 0, lEncoder, lMotor);
	PIDController rController = new PIDController(.05, 0, 0, rEncoder, rMotor);
	
	PIDController driveController = new PIDController(.015, 0, 0, lEncoder, driveMotors);
	
	double distancePerPulse = Math.PI * 6 /360;
	
	@Override
	public void robotInit() {
		SmartDashboard.putNumber("p", p);
		SmartDashboard.putNumber("i", i);
		SmartDashboard.putNumber("d", d);
		System.out.println("[  STATUS  ] Initiating...");
		spiGyro.calibrate();
		System.out.println("[  STATUS  ] Initialized.");
		
		rEncoder.setReverseDirection(true);
		
		lEncoder.setDistancePerPulse(distancePerPulse);
		rEncoder.setDistancePerPulse(distancePerPulse);
		
		lController.setAbsoluteTolerance(2);
		rController.setAbsoluteTolerance(2);
		
		driveController.setAbsoluteTolerance(2);
		driveController.setOutputRange(-0.6, 0.6);
		
		rController.setOutputRange(-0.4, 0.4);
		lController.setOutputRange(-0.4, 0.4);
		
		lEncoder.reset();
		rEncoder.reset();
		
	}
	
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	
	@Override
	public void autonomousInit() {
		lController.setP(SmartDashboard.getNumber("p", 0));
		lController.setI(SmartDashboard.getNumber("i", 0));
		lController.setD(SmartDashboard.getNumber("d", 0));
		rController.setP(SmartDashboard.getNumber("p", 0));
		rController.setI(SmartDashboard.getNumber("i", 0));
		rController.setD(SmartDashboard.getNumber("d", 0));
		driveController.setP(SmartDashboard.getNumber("p", 0));
		driveController.setI(SmartDashboard.getNumber("i", 0));
		driveController.setD(SmartDashboard.getNumber("d", 0));

		timer.reset();
		timer.start();
		spiGyro.reset();
		
		lEncoder.reset();
		rEncoder.reset();
		
		lMotor.setInverted(true);
		
		lController.setSetpoint(16);
		rController.setSetpoint(-16);
		
//		driveController.setSetpoint(190);
		
		lController.enable();
		rController.enable();
		
//		driveController.enable();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
//		System.out.print("R" + rController.getError());
//		System.out.print("R" + rEncoder.getDistance());
//		System.out.print("L" + lController.getError());
//		System.out.print("L" + lEncoder.getDistance());
		SmartDashboard.updateValues();
		SmartDashboard.putNumber("encoder l", lEncoder.getDistance());
		SmartDashboard.putNumber("encoder r", rEncoder.getDistance());
		
		if (lController.onTarget()) {
			lController.disable();
			System.out.println("L Controller Disabled");
		}
		if (rController.onTarget()) { 
			rController.disable();
			System.out.println("R Controller Disabled");
		}
	}	
	
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	
	@Override
	public void teleopInit() {
		lController.disable();
		rController.disable();
		driveController.disable();
		
		lEncoder.reset();
		rEncoder.reset();
		lMotor.setInverted(false);
	}

	/**
	 *	This function is called periodically during operator control
	*/
	
	@Override
	public void teleopPeriodic() {
		
		speedControl();
		speedControlRotate();
		motorLift();
		motorGear();
		oneEighty();
		SmartDashboard.putNumber("encoder l", lEncoder.getDistance());
		SmartDashboard.putNumber("encoder r", rEncoder.getDistance());


		myRobot.arcadeDrive(stick0.getRawAxis(1)*speedLimitMove, stick0.getRawAxis(4)*speedLimitRotate);
	}
	
	
	/**
	 * This function is called periodically during test mode
	 */
	
	@Override
	public void testPeriodic() {
	}	
	
	public void motorLift() {
		if(stick0.getRawButton(12)){
			motorLift.setSpeed(-1.0); // Towards forward
		}
		if(!(stick0.getRawButton(12))){
			motorLift.setSpeed(0.0);
		}
	}
	
	public void motorGear() {
		if(stick0.getRawButton(11)) {
			if(stick0.getRawButton(1)){
				motorGear.setSpeed(-1.0); // Open gear holder
			}
			if(stick0.getRawButton(2)){
				motorGear.setSpeed(1.0); // Close gear holder
			}
			if(stick0.getRawButton(1)==(stick0.getRawButton(2))) {
				motorGear.setSpeed(0.0); // Stop gear holder
			}
		}
		if(!(stick0.getRawButton(11))) {
			if(stick0.getRawButton(1)){
				timer.reset();
				timer.start();
				if(timer.get()<1.25){
					motorGear.setSpeed(-1.0);
				}
			}
			if(stick0.getRawButton(2)) {
				timer.reset();
				timer.start();
				if(timer.get()<1.5){
					motorGear.setSpeed(1.0);
				}
			}
			if(stick0.getRawButton(1)==(stick0.getRawButton(2))) {
				motorGear.setSpeed(0.0);
			}
		}

	}
	
	public void speedControl() {
		if(stick0.getRawButton(3)){
			speedLimitMove = 0.5;
		}
		if(stick0.getRawButton(4)){
			speedLimitMove = 0.6;
		}
		if(stick0.getRawButton(5)){
			speedLimitMove = 0.8;
		}
		if(stick0.getRawButton(6)){
			speedLimitMove = 1;	
		}
	}
	
	public void speedControlRotate() {
		if(stick0.getRawButton(7)){
			speedLimitRotate = 0.5;
		}
		if(stick0.getRawButton(8)){
			speedLimitRotate= 0.6;
		}
		if(stick0.getRawButton(9)){
			speedLimitRotate = 0.7;
		}
		if(stick0.getRawButton(10)){
			speedLimitRotate = 1;	
		}
	}
	
	public void oneEighty() {
		if(stick0.getPOV(0)==180){
			double rotatedHeading = spiGyro.getAngle()+180;
			while(spiGyro.getAngle()<rotatedHeading){
				System.out.println("[GYRO ANGLE] " + spiGyro.getAngle());
				myRobot.arcadeDrive(0, speedLimitRotate);
				if (stick0.getPOV(0)==0){
					myRobot.arcadeDrive(0, 0);
					break;
				}
			}
		}
	}
}
