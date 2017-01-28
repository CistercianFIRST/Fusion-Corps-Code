package org.usfirst.frc.team6672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
		
	RobotDrive myRobot = new RobotDrive(1, 0);
	Timer timer = new Timer();
	
	
	/* Joystick stuff */
	Joystick stick0 = new Joystick(0);

	/* Speed Control System */
	double speedLimitMove = 0.6;
	double speedLimitRotate = -0.6;		// Has to be negative bc the joystick inverts l/r
	
	/* Gyro Systems */
	ADXRS450_Gyro spiGyro = new ADXRS450_Gyro();
	//AnalogGyro gyro = new AnalogGyro(1);
	double Kp = 0.03*.35;					//Gyro converter constant
	
	
	/**
	 * This function is run when the robot is first started up and should 
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		spiGyro.reset();
		spiGyro.calibrate();
		
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
		if (timer.get() < 5.0) {
			double angle = spiGyro.getAngle();
			myRobot.drive(-0.4, angle*Kp);	// drive forwards half speed, and correct heading with gyro
			
			System.out.println(spiGyro.getAngle());
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
		if(stick0.getRawButton(3)){
			speedLimitMove = 0.4;
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
		 myRobot.arcadeDrive(stick0.getRawAxis(1)*speedLimitMove, stick0.getRawAxis(0)*speedLimitRotate);
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
