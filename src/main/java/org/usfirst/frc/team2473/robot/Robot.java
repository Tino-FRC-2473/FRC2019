/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.robot.commands.StraightDrive;
import org.usfirst.frc.team2473.robot.commands.TeleopDrive;
import org.usfirst.frc.team2473.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team2473.robot.subsystems.SparkDriveSubsystem;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
	
	public static DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
	public static SparkDriveSubsystem sparkDriveSubsystem = SparkDriveSubsystem.getInstance();
	
	public static Relay cvLight;
	
	public static OI oi;

	Preferences prefs;

	/**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		cvLight = new Relay(0);
		prefs = Preferences.getInstance();
				
		// UsbCamera cubeCam = CameraServer.getInstance().startAutomaticCapture("Cube View", 0);
		// cubeCam.setBrightness(75);
		// cubeCam.setResolution(640, 480);
		// UsbCamera driveCam = CameraServer.getInstance().startAutomaticCapture("Front View", 1);
		// driveCam.setBrightness(75);
		// driveCam.setResolution(640, 480);
		
		Devices.getInstance().getNavXGyro().reset();
	}
	
	/**
	 * Runs once each time the robot is set to disabled
	 */
	@Override
	public void disabledInit() {
		//cvLight.set(Relay.Value.kOff);
		System.out.println("AFTER DISABLED: " + Devices.getInstance().getNavXGyro().getAngle());
		Scheduler.getInstance().removeAll();
		
	}

	/**
	 * Runs continuously while the robot is in the disabled state
	 */
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the autonomous state
	 */
	@Override
	public void autonomousInit() {
		
		// Turn on the CV light
		// cvLight.set(Relay.Value.kReverse);
		
// 		driveSubsystem.resetEncoders();
// //	
		
// 		System.out.println("Starting align to hatch ------------------------------------");
		
		//new StraightDrive(24, 0.3).start();
		
//		int distanceFirst  = prefs.getInt("1. First Distance", 48);
//		int degrees  = prefs.getInt("2. Turn Degrees", 180);
//		int distanceSecond  = prefs.getInt("3. Second Distance", 48);
//				
//		AutonomousTester tester = new AutonomousTester();
//		tester.addDriveTurnDrive(distanceFirst, degrees, distanceSecond);
//		tester.start();

		new StraightDrive(20, 0.3).start();

	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {
        JetsonPort.getInstance().updateVisionValues();	
        System.out.println("Vision - Angle: " + JetsonPort.getInstance().getVisionAngle() + 
            " Distance: " + JetsonPort.getInstance().getVisionDistance());
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the teleop state
	 */
	@Override
	public void teleopInit() {
		cvLight.set(Value.kForward);
		(new TeleopDrive()).start();
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {
        JetsonPort.getInstance().updateVisionValues();
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
	
}