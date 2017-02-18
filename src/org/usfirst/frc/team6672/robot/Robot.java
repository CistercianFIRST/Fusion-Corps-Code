package org.usfirst.frc.team6672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
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
	int testVariable = 1;
	/* Joystick stuff */
	Joystick stick0 = new Joystick(0);

	/* Speed Control System */
	double speedLimitMove = 0.6;
	double speedLimitRotate = -0.5;		// Has to be negative bc the joystick inverts l/r
	
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
		System.out.println("[ POV input is " + stick0.getPOV(0));
		System.out.println("The number of POV's is " + stick0.getPOVCount());
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
		if (timer.get() < 3.0) {
			double angle = spiGyro.getAngle();
			myRobot.drive(-0.4, angle*Kp);	// drive forwards half speed, and correct heading with gyro
			
			System.out.println("[GYRO ANGLE] " + spiGyro.getAngle());
		} else {
			myRobot.drive(0.0, 0.0);		// stop robot
		}
	}	
	
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	
	@Override
	public void teleopInit() {
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

		myRobot.arcadeDrive(stick0.getRawAxis(1)*speedLimitMove, stick0.getRawAxis(0)*speedLimitRotate);
	}
	
	
	/**
	 * This function is called periodically during test mode
	 */
	
	@Override
	public void testPeriodic() {
	}	
	
	public void cameraInit() {
        CameraServer.getInstance().startAutomaticCapture("Back Camera", 0).setResolution(320, 240);
        CameraServer.getInstance().startAutomaticCapture("Front Camera", 1).setResolution(512, 288);
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
				while(timer.get()<1.5){
					motorGear.setSpeed(-1.0);
				}
			}
			if(stick0.getRawButton(2)) {
				timer.reset();
				timer.start();
				while(timer.get()<1.25){
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
			speedLimitRotate = -0.5;
		}
		if(stick0.getRawButton(8)){
			speedLimitRotate= -0.6;
		}
		if(stick0.getRawButton(9)){
			speedLimitRotate = -0.7;
		}
		if(stick0.getRawButton(10)){
			speedLimitRotate = -1;	
		}
	}
	
	public void oneEighty() {
		if(stick0.getPOV(0)==180){
			double rotatedHeading = spiGyro.getAngle()+180;
			while(spiGyro.getAngle()<rotatedHeading){
				System.out.println("[GYRO ANGLE] " + spiGyro.getAngle());
				myRobot.arcadeDrive(0, speedLimitRotate);
				if (stick0.getPOV(0)==0){
					myRobot.drive(0, 0);
					break;
				}
			}
		}
	}
}
