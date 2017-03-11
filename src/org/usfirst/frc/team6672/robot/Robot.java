package org.usfirst.frc.team6672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	
	/** Inital Variables **/

	RobotDrive myRobot = new RobotDrive(1, 0);
	Timer timer = new Timer();
	Timer timerTwo = new Timer();

	/* Joystick stuff */
	Joystick stick0 = new Joystick(0);
	int stick0POV = -1;
	double stick0Axis1 = 0;
	double stick0Axis2 = 0;
	double stick0Axis3 = 0;
	double stick0Axis4 = 0;
	boolean stick0Button1 = false;
	boolean stick0Button2 = false;
	boolean stick0Button3 = false;
	boolean stick0Button4 = false;
	boolean stick0Button5 = false;
	boolean stick0Button6 = false;
	boolean stick0Button7 = false;

	/* Speed Control System */
	double speedLimitMove = 1.0;
	double speedLimitRotate = -0.7;		// Has to be negative bc the joystick inverts l/r
	
	/* Sensor Systems */
	ADXRS450_Gyro spiGyro = new ADXRS450_Gyro();
	double Kp = 0.03*.35;				//Gyro converter constant (corrected)
	
	/* PWM Stuff */
	Spark motorLift = new Spark(2);
	Spark motorGear = new Spark(3);
	
	/**
	 * This function is run when the robot is first started up and should 
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		System.out.println("[  STATUS  ] Initiating...");
		spiGyro.calibrate();
		cameraInit();
		System.out.println("[  STATUS  ] Initialized.");
	}
	
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		spiGyro.reset();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
		DriverStation.reportError("[   MODE   ] Autonomous...", true);
		
		if(timer.get() < 3.0) {
			double angle = spiGyro.getAngle();
			myRobot.drive(-0.4, angle*Kp);
		}
		else {
			myRobot.drive(0.0, 0.0);
		}
	}
	
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	
	@Override
	public void teleopInit() {
		timer.reset();
		timer.start();
	}

	/**
	 *	This function is called periodically during operator control
	*/
	
	@Override
	public void teleopPeriodic() {
		DriverStation.reportError("[   MODE   ] Teleop...", true);
		
		stick0POV = stick0.getPOV(0);
		stick0Axis1 = stick0.getRawAxis(1);
		stick0Axis2 = stick0.getRawAxis(2);
		stick0Axis3 = stick0.getRawAxis(3);
		stick0Axis4 = stick0.getRawAxis(4);
		stick0Button1 = stick0.getRawButton(1);
		stick0Button2 = stick0.getRawButton(2);
		stick0Button3 = stick0.getRawButton(3);
		stick0Button4 = stick0.getRawButton(4);
		stick0Button5 = stick0.getRawButton(5);
		stick0Button6 = stick0.getRawButton(6);
		stick0Button7 = stick0.getRawButton(7);
		
		speedControl();
		speedControlRotate();
		motorLift();
		motorGear();
		
		myRobot.arcadeDrive(stick0Axis1*speedLimitMove, stick0Axis4*speedLimitRotate);
	}
	
	
	/**
	 * This function is called periodically during test mode
	 */
	
	@Override
	public void testPeriodic() {
	}	
	
	public void cameraInit() {
        //CameraServer.getInstance().startAutomaticCapture("Back Camera", 0).setResolution(128, 72);
        CameraServer.getInstance().startAutomaticCapture("Front Camera", 0).setResolution(256, 144);
	}
	
	public void motorLift() {
		if(stick0Button4){
			motorLift.setSpeed(-1.0); // Towards forward
		}
		if(!(stick0Button4)){
			motorLift.setSpeed(0.0);
		}
	}
	
	public void motorGear() {
		if(stick0Button7) {
			if(stick0Axis3>0.5){
				motorGear.setSpeed(-1.0); // Open gear holder
			}
			if(stick0Axis2>0.5){
				motorGear.setSpeed(1.0); // Close gear holder
			}
			if(stick0Axis3<=0.5 && stick0Axis2<=0.5) {
				motorGear.setSpeed(0.0); // Stop gear holder
			}
		}
		if(!(stick0Button7)) {
			if(stick0Axis3>0.5){
				timer.reset();
				timer.start();
				if(timer.get()<1.5){
					motorGear.setSpeed(-1.0);
				}
			}
			if(stick0Axis2>0.5) {
				timer.reset();
				timer.start();
				if(timer.get()<1.25){
					motorGear.setSpeed(1.0);
				}
			}
			if(stick0Axis3<=0.5 && stick0Axis2<=0.5) {
				motorGear.setSpeed(0.0);
			}
		}
	}
	
	public void speedControl() {
		if(stick0Button5){
			speedLimitMove = 0.8;
		}
		if(stick0Button6){
			speedLimitMove = 1;	
		}
	}
	
	public void speedControlRotate() {
		if(stick0Button3){
			speedLimitRotate= -0.6;
		}
		if(stick0Button2){
			speedLimitRotate = -0.7;
		}
		if(stick0Button1){
			speedLimitRotate = -1;	
		}
	}
}
