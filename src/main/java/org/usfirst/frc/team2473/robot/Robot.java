/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.robot.commands.AlignToHatch;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

import java.util.Map;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Robot extends TimedRobot {
	
	public static SparkDriveSubsystem driveSubsystem = null; 
	//TODO sketchy temporary fix to SparkDriveSubsystem not on robot
	
    public static Relay cvLight;
	
	public static OI oi;

	Preferences prefs;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Board");
	// private static NetworkTableEntry timerEntry;
	// public static NetworkTableEntry angleEntry;
	// public static NetworkTableEntry distanceEntry;
	private Timer timer = new Timer();


	/**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		//angleEntry = tab.add("angle","-90").withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90)).getEntry();
		//timerEntry = tab.add("Time","0").withWidget("Text View").getEntry();
		//timerEntry = tab.add("Time","0").withSize(1,1).withPosition(0,0).getEntry();
		//distanceEntry = tab.add("distance","0").withSize(1, 1).withPosition(2, 2).getEntry();
		//entry = SmartDashboard.getEntry("time");
		oi = new OI();
		
		cvLight = new Relay(0);
        prefs = Preferences.getInstance();
        
		Devices.getInstance().getNavXGyro().reset();
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
		camera1.setResolution(240, 240);
		camera1.setFPS(20);
		camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		camera1.setVideoMode(PixelFormat.kMJPEG,240,240,20);
		
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setResolution(240, 240);
		camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		camera2.setExposureManual(30);
		camera2.setBrightness(20);
		camera2.setFPS(20);
		camera2.setVideoMode(PixelFormat.kMJPEG,240,240,20);
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
		
		new AlignToHatch().start(); //run the code to start CV
	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {

		//Calculates the min and sec left based on t and updates shuffleboard
		double t = 15-timer.get();
		if(t<0) t= 0;
		String min = ""+(int)(t/60);
		String sec = ""+(int)(t%60);
		if(sec.length()==1) sec = "0"+sec;
		SmartDashboard.putString("Time", min+":"+sec);


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
		//(new TeleopDrive()).start();
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {

		//Calculates the min and sec left based on t and updates shuffleboard
		double t = 135-timer.get();
		if(t<0) t= 0;
		String min = ""+(int)(t/60);
		String sec = ""+(int)(t%60);
		if(sec.length()==1) sec = "0"+sec;
		SmartDashboard.putString("Time", min+":"+sec);


        // JetsonPort.getInstance().updateVisionValues();
        // System.out.println("Vision - Angle: " + JetsonPort.getInstance().getVisionAngle() + 
        // " Distance: " + JetsonPort.getInstance().getVisionDistance());
        
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
	
}