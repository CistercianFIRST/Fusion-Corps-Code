package org.usfirst.frc.team6672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.PIDController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	
	RobotDrive myRobot = new RobotDrive(1, 0);
	Joystick stick0 = new Joystick(0);
	Timer timer = new Timer();
	
	// Speed Control System
	boolean turbo = false; // When turbo is false speedLimit is active
	double speedLimit = 0.5;
	
	// Gyro Systems
	AnalogGyro gyro = new AnalogGyro(1);
	double Kp = 0.03;     //Gyro converter constant
	
	/**
	 * This function is run when the robot is first started up and should 
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		gyro.reset();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (timer.get() < 2.0) {
			double angle = gyro.getAngle();
			myRobot.drive(-0.5, -angle*Kp); // drive forwards half speed, and correct heading with gyro
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
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
		//myRobot.setMaxOutput(0.5); // Doesn't work for both motors
		if(turbo = true){
			speedLimit = 1.0;
		}
		myRobot.tankDrive(stick0.getRawAxis(1)*speedLimit, stick0.getRawAxis(5)*speedLimit);
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
