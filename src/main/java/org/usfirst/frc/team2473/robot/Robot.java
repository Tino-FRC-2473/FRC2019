/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.robot.commands.AlignToHatch;
import org.usfirst.frc.team2473.robot.commands.AutonomousTester;
import org.usfirst.frc.team2473.robot.commands.ElevatorTicksTest;
import org.usfirst.frc.team2473.robot.commands.StraightDrive;
import org.usfirst.frc.team2473.robot.commands.TeleopDrive;
import org.usfirst.frc.team2473.robot.commands.TeleopDrive2;
import org.usfirst.frc.team2473.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team2473.robot.subsystems.Elevator;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
	
	public static DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    public static Elevator elevator = Elevator.getInstance();
    
	public static Relay cvLight;
	
	public static OI oi;

	Preferences prefs;

	/**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		// cvLight = new Relay(RobotMap.CV_LIGHT);
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
        //Devices.getInstance().initializeCVSocket();
		
		// Turn on the CV light
		// cvLight.set(Relay.Value.kReverse);
		
// 		driveSubsystem.resetEncoders();
// //	
		
// 		System.out.println("Starting align to hatch ------------------------------------");
		
		//new StraightDrive(24, 0.3).start();
		//new AlignToHatch().start();

		elevator.resetEncoders();
		//new ElevatorTicksTest().start();
		new TeleopDrive2(false).start();
		
//		int distanceFirst  = prefs.getInt("1. First Distance", 48);
//		int degrees  = prefs.getInt("2. Turn Degrees", 180);
//		int distanceSecond  = prefs.getInt("3. Second Distance", 48);
//				
//		AutonomousTester tester = new AutonomousTester();
//		tester.addDriveTurnDrive(distanceFirst, degrees, distanceSecond);
//		tester.start();

	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {		
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the teleop state
	 */
	@Override
	public void teleopInit() {
		//Devices.getInstance().initializeCVSocket();
		System.out.println("STARTING ELEVATOR -----------------------------------");
		elevator.resetEncoders();
		System.out.println("ENCODERS RESET");
		new TeleopDrive2(true).start();
		
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {
		System.out.println("Running Elevator " + Robot.oi.getWheel().getX());

		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {
		elevator.resetEncoders();
		new TeleopDrive2(true).start();
	}

	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
	}
	
}