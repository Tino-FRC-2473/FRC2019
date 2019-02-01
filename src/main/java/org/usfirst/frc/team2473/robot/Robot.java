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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Robot extends TimedRobot {
	
	public static SparkDriveSubsystem driveSubsystem = null; 
	//TODO sketchy temporary fix to SparkDriveSubsystem not on robot
	
    public static Relay cvLight;
	
	public static OI oi;

	Preferences prefs;

	private NetworkTableEntry entry;

	private Timer timer = new Timer();

	/**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		entry = SmartDashboard.getEntry("time");
		oi = new OI();
		
		cvLight = new Relay(0);
        prefs = Preferences.getInstance();
        
		Devices.getInstance().getNavXGyro().reset();

		//TODO sketchy code that somehow made the camera work
		new Thread(() -> {
			UsbCamera camera =CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
			
			Mat source = new Mat();
			Mat output = new Mat();
			
			while(!Thread.interrupted()) {
				cvSink.grabFrame(source);
				//Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();
	}
	
	/**
	 * Runs once each time the robot is set to disabled
	 */
	@Override
	public void disabledInit() {
       // driveSubsystem.drive(0, 0);
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
		timer.reset();
		timer.start();
		
	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {

		double t = 15-timer.get();
		if(t<0) t= 0;
		String min = ""+(int)(t/60);
		String sec = ""+(int)(t%60);
		if(sec.length()==1) sec = "0"+sec;
		entry.setString(min+":"+sec);


        //JetsonPort.getInstance().updateVisionValues();	
        // System.out.println("Vision - Angle: " + JetsonPort.getInstance().getVisionAngle() + 
        //     " Distance: " + JetsonPort.getInstance().getVisionDistance());
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the teleop state
	 */
	@Override
	public void teleopInit() {
		timer.reset();
        //cvLight.set(Value.kForward);
        //JetsonPort.getInstance().updateVisionValues();	
        //System.out.println("Vision - " + JetsonPort.getInstance().getVisionAngle() + JetsonPort.getInstance().getVisionDistance());
		(new TeleopDrive()).start();
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {
		double t = 135-timer.get();
		if(t<0) t= 0;
		String min = ""+(int)(t/60);
		String sec = ""+(int)(t%60);
		if(sec.length()==1) sec = "0"+sec;
		entry.setString(min+":"+sec);


        // JetsonPort.getInstance().updateVisionValues();
        // System.out.println("Vision - Angle: " + JetsonPort.getInstance().getVisionAngle() + 
        // " Distance: " + JetsonPort.getInstance().getVisionDistance());
        
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
	
}