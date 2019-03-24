/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.robot.commands.AlignToHatch;
import org.usfirst.frc.team2473.robot.commands.ArmZero;
import org.usfirst.frc.team2473.robot.commands.AutonomousTester;
import org.usfirst.frc.team2473.robot.commands.ElevatorZero;
import org.usfirst.frc.team2473.robot.commands.TeleopDrive;
import org.usfirst.frc.team2473.robot.subsystems.Arm;
import org.usfirst.frc.team2473.robot.subsystems.Cargo;
import org.usfirst.frc.team2473.robot.subsystems.Elevator;
import org.usfirst.frc.team2473.robot.subsystems.Roller;
import org.usfirst.frc.team2473.robot.subsystems.SparkDriveSubsystem;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	public static SparkDriveSubsystem driveSubsystem = SparkDriveSubsystem.getInstance();
	public static Cargo cargo = Cargo.getInstance();
    public static Elevator elevator = Elevator.getInstance();
    public static Arm arm = Arm.getInstance();
    public static Roller roller = Roller.getInstance();

	TeleopDrive teleopDrive;
	public static Relay cvLight;

	public static OI oi;

	public static JetsonPort jetsonPort;

	private int i = 0;

	Preferences prefs;
    Thread m_visionThread;
    
    // NetworkTableEntry angle1 = Shuffleboard.getTab("Drive").add("Angle 1", 0).getEntry();
    // NetworkTableEntry angle2 = Shuffleboard.getTab("Drive").add("Angle 2", 0).getEntry();
    // NetworkTableEntry angle3 = Shuffleboard.getTab("Drive").add("Angle 3", 0).getEntry();
    // NetworkTableEntry distance1 = Shuffleboard.getTab("Drive").add("Distance 1", 0).getEntry();
    // NetworkTableEntry distance2 = Shuffleboard.getTab("Drive").add("Distance 2", 0).getEntry();
    // NetworkTableEntry distance3 = Shuffleboard.getTab("Drive").add("Distance 3", 0).getEntry();
    
    NetworkTableEntry manualControlEntry = Shuffleboard.getTab("Drive").add("Manual Control", RobotMap.MANUAL_CONTROL).getEntry();
    NetworkTableEntry currentPositionEntry = Shuffleboard.getTab("Drive").add("Current Position", ElevatorPosition.ZERO.toString()).getEntry();
    NetworkTableEntry lastPressedEntry = Shuffleboard.getTab("Drive").add("Last Pressed Position", getLastPressedPositionString()).getEntry();
    NetworkTableEntry cargoSecuredEntry = Shuffleboard.getTab("Drive").add("Cargo Secured", false).getEntry();

    NetworkTableEntry cvRunningEntry = Shuffleboard.getTab("Drive").add("CV Running", RobotMap.CV_RUNNING).getEntry();
    NetworkTableEntry scoringHatchEntry = Shuffleboard.getTab("Drive").add("Scoring Hatch", RobotMap.SCORING_HATCH).getEntry();
    NetworkTableEntry elevatorOutputCurrentEntry = Shuffleboard.getTab("Drive").add("Elevator Output Current", elevator.spark.getSparkMaxObject().getOutputCurrent()).withWidget(BuiltInWidgets.kGraph).getEntry();

	@Override
	public void robotInit() {
		oi = new OI();

		teleopDrive = new TeleopDrive(true); // ?

        cvLight = new Relay(0);
		prefs = Preferences.getInstance();

		try {
			jetsonPort = new JetsonPort(9600, Port.kUSB);
			RobotMap.CV_RUNNING = true;
		} catch (Exception e) {
			System.out.println("ERROR: " + e.getClass());
			RobotMap.CV_RUNNING = false;
		}

		Devices.getInstance().getNavXGyro().reset();

		UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture("Front Camera", 0);
		frontCam.setBrightness(25);
		frontCam.setFPS(15);
        frontCam.setResolution(160, 120);
        
        UsbCamera armCam = CameraServer.getInstance().startAutomaticCapture("Arm Camera", 1);
		armCam.setBrightness(25);
		armCam.setFPS(15);
		armCam.setResolution(160, 120);

		m_visionThread = new Thread(() -> {

			CvSink cvSinkFront = CameraServer.getInstance().getVideo(frontCam);
			CvSource outputStreamFront = CameraServer.getInstance().putVideo("Front Camera", 160, 120);
			Mat matFront = new Mat();

			while (!Thread.interrupted()) {

                double x1 = -99;
                double x2 = -99;
                double x3 = -99;

                if (RobotMap.CV_RUNNING) {
                    x1 = jetsonPort.getVisionX1()/2.0;
                    x2 = jetsonPort.getVisionX2()/2.0;
                    x3 = jetsonPort.getVisionX3()/2.0;
                }
				

				double alignX = AlignToHatch.x / 2.0;

				Scalar alignColor;

				if (AlignToHatch.isRunning) {
					alignColor = new Scalar(0, 255, 0);
				} else {
					alignColor = new Scalar(59, 214, 214);
				}

				if (cvSinkFront.grabFrame(matFront) == 0) {
					outputStreamFront.notifyError(cvSinkFront.getError());
				} else {
					
					double shift = 3;

					Imgproc.line(matFront, new Point(x1 + shift, 0), new Point(x1 + shift, 120), new Scalar(255, 0, 0), 1);
					Imgproc.line(matFront, new Point(x2 + shift, 0), new Point(x2 + shift, 120), new Scalar(255, 0, 0), 1);
					Imgproc.line(matFront, new Point(x3 + shift, 0), new Point(x3 + shift, 120), new Scalar(255, 0, 0), 1);
					Imgproc.line(matFront, new Point(alignX + shift, 0), new Point(alignX + shift, 120), alignColor, 1);
					
					outputStreamFront.putFrame(matFront);
					
				}	
				
			}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();
		
	}

	@Override
	public void disabledInit() {
        teleopDrive.cancel();

		driveSubsystem.drive(0, 0);
		Scheduler.getInstance().removeAll();

	}

	@Override
	public void disabledPeriodic() {
        updateShuffleboardVisualizations();
        jetsonPort.updateVisionValues();
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		cvLight.set(Value.kForward);
        new ElevatorZero().start();
        new ArmZero().start();
        teleopDrive = new TeleopDrive(true);
        teleopDrive.start();
	}

	@Override
	public void autonomousPeriodic() {
        updateShuffleboardVisualizations();
        System.out.println(elevator.getEncoderTicks());
        jetsonPort.printVisionAngles();
        jetsonPort.updateVisionValues();

		// if (RobotMap.CV_RUNNING) {
                
        //     double angle = jetsonPort.getVisionAngle1();
        //     double distance = jetsonPort.getVisionDistance1();
        //     double x1 = 29.75;

        //     double x2 = distance - x1;
        //     double h = distance * Math.tan(Math.toRadians(angle));

        //     angle = (int) Math.toDegrees(Math.atan(h/x2));
        // }

		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
        cvLight.set(Value.kForward);
        teleopDrive = new TeleopDrive(false);
        teleopDrive.start();
        // AutonomousTester t = new AutonomousTester();
        // t.addDiagnosticTests();
        // t.start();
	}

	@Override
	public void teleopPeriodic() {
        autonomousPeriodic();
        // Scheduler.getInstance().run();
	}

    @Override
    public void testInit() {
        AutonomousTester t = new AutonomousTester();
        t.addDiagnosticTests();
        t.start();
    }

	@Override
	public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    private String getLastPressedPositionString() {
        if (TeleopDrive.lastPressedPosition == null) return "STOW";
        return "Elevator: " + TeleopDrive.lastPressedPosition.toString() + "\n"
                + "Arm: " + TeleopDrive.getArmPositionFromElevator();
    }

    private String getCurrentPositionString() {
        if (TeleopDrive.lastPressedPosition == null) return "STOW";
        return "Elevator: " + elevator.getExecutingGoalPosition().toString() + "\n"
                + "Arm: " + arm.getExecutingGoalPosition().toString();
    }
    
    public void updateShuffleboardVisualizations() {

        // RobotMap.CAMERAS_SWITCHED = switchedCameras.getBoolean(false);        
        // System.out.println(RobotMap.MANUAL_CONTROL);

        // angle1.setDouble(jetsonPort.getVisionAngle1());
        // angle2.setDouble(jetsonPort.getVisionAngle2());
        // angle3.setDouble(jetsonPort.getVisionAngle3());
        // distance1.setDouble(jetsonPort.getVisionDistance1());
        // distance2.setDouble(jetsonPort.getVisionDistance2());
        // distance3.setDouble(jetsonPort.getVisionDistance3());

        currentPositionEntry.setString(getCurrentPositionString());

        cargoSecuredEntry.setBoolean(Robot.cargo.getState() == Robot.cargo.CAPTURING);
		
        lastPressedEntry.setString(getLastPressedPositionString());
        
        scoringHatchEntry.setBoolean(RobotMap.SCORING_HATCH);
        cvRunningEntry.setBoolean(RobotMap.CV_RUNNING);
        elevatorOutputCurrentEntry.setDouble(elevator.spark.getSparkMaxObject().getOutputCurrent());

        manualControlEntry.setBoolean(RobotMap.MANUAL_CONTROL);
        SmartDashboard.updateValues();
    }

}