/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.robot.commands.TeleopDrive;
import org.usfirst.frc.team2473.robot.subsystems.SparkDriveSubsystem;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
	
	public static SparkDriveSubsystem driveSubsystem = SparkDriveSubsystem.getInstance();
	
    public static Relay cvLight;
	
	public static OI oi;

	private int i = 0;

	Preferences prefs;

	SerialPort serialPort;

	/**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		cvLight = new Relay(0);
        prefs = Preferences.getInstance();
        
		Devices.getInstance().getNavXGyro().reset();

		UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture("Front Camera", 0);
		frontCam.setBrightness(25);
		frontCam.setFPS(15);
		frontCam.setResolution(320, 240);
		UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture("Back Camera", 1);
		backCam.setBrightness(75);
		backCam.setFPS(15);
		backCam.setResolution(320, 240);

		// serialPort = new SerialPort(9600, SerialPort.Port.kOnboard);
	}
	
	/**
	 * Runs once each time the robot is set to disabled
	 */
	@Override
	public void disabledInit() {
        driveSubsystem.drive(0, 0);
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
		cvLight.set(Value.kForward);
	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {
		// serialPort.writeString("Hello World!");

        // JetsonPort.getInstance().updateVisionValues();	
		// if (++i % 4 == 0) {
		// 	JetsonPort.getInstance().printVisionAngles();
		// }
		//System.out.println("Hello World!");
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the teleop state
	 */
	@Override
	public void teleopInit() {
        cvLight.set(Value.kForward);
		JetsonPort.getInstance().updateVisionValues();
		JetsonPort.getInstance().printVisionAngles();
		(new TeleopDrive()).start();
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {
        JetsonPort.getInstance().updateVisionValues();
		
		if (++i % 4 == 0) {
			JetsonPort.getInstance().printVisionAngles();
		}

        
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
	
}