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
import org.usfirst.frc.team2473.robot.commands.ElevatorZero;
import org.usfirst.frc.team2473.robot.commands.StraightDrive;
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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
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
    
    NetworkTableEntry switchedCameras = Shuffleboard.getTab("Drive").add("Cameras Switched", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    NetworkTableEntry angle1 = Shuffleboard.getTab("Drive").add("Angle 1", 0).getEntry();
    NetworkTableEntry angle2 = Shuffleboard.getTab("Drive").add("Angle 2", 0).getEntry();
    NetworkTableEntry angle3 = Shuffleboard.getTab("Drive").add("Angle 3", 0).getEntry();
    NetworkTableEntry distance1 = Shuffleboard.getTab("Drive").add("Distance 1", 0).getEntry();
    NetworkTableEntry distance2 = Shuffleboard.getTab("Drive").add("Distance 2", 0).getEntry();
    NetworkTableEntry distance3 = Shuffleboard.getTab("Drive").add("Distance 3", 0).getEntry();
    
    NetworkTableEntry elevatorPosition = Shuffleboard.getTab("Drive").add("Elevator Position", ElevatorPosition.ZERO.toString()).getEntry();
    NetworkTableEntry cargoState = Shuffleboard.getTab("Drive").add("Cargo State", "Rearming").getEntry();
    NetworkTableEntry cargoSecured = Shuffleboard.getTab("Drive").add("Cargo Secured", false).getEntry();
    NetworkTableEntry lastPressedEntry = Shuffleboard.getTab("Drive").add("Last Pressed", ElevatorPosition.ZERO.toString()).getEntry();

    NetworkTableEntry cvRunning = Shuffleboard.getTab("Drive").add("CV Running", RobotMap.CV_RUNNING).getEntry();
    NetworkTableEntry elevatorMoving = Shuffleboard.getTab("Drive").add("Elevator Moving", Robot.elevator.isMoving()).getEntry();

    /**
	 * Runs once when the robot turns on
	 */
	@Override
	public void robotInit() {
		oi = new OI();

		teleopDrive = new TeleopDrive();

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

		UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture("Back Camera", 1);
		backCam.setBrightness(25);
		backCam.setFPS(15);
        backCam.setResolution(160, 120);
        
        //SmartDashboard.putBoolean("Cameras Switched", false);

		m_visionThread = new Thread(() -> {

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSinkFront = CameraServer.getInstance().getVideo(frontCam);
			CvSink cvSinkBack = CameraServer.getInstance().getVideo(backCam);
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStreamFront = CameraServer.getInstance().putVideo("Front Camera", 160, 120);
			CvSource outputStreamBack = CameraServer.getInstance().putVideo("Back Camera", 160, 120);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat matFront = new Mat();
			Mat matBack = new Mat();

			// Add a test point to each camera to get both feeds up
			
			// if (cvSinkFront.grabFrame(matFront) == 0) {
			// 	// Send the output the error.
			// 	outputStreamFront.notifyError(cvSinkFront.getError());
			// }
			// if (cvSinkBack.grabFrame(matBack) == 0) {
			// 	// Send the output the error.
			// 	outputStreamBack.notifyError(cvSinkBack.getError());
			// }

			// Imgproc.line(matFront, new Point(0,0), new Point(1,1), new Scalar(0, 0, 255));
			// Imgproc.line(matBack, new Point(0,0), new Point(1,1), new Scalar(0, 0, 255));

			// outputStreamFront.putFrame(matFront);
			// outputStreamBack.putFrame(matBack);

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
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

				//System.out.println(x1 + " " + x2 + " " + x3 + " " + alignX);
					
				// ------------ FRONT ----------------

				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSinkFront.grabFrame(matFront) == 0) {
					// Send the output the error.
					outputStreamFront.notifyError(cvSinkFront.getError());
				} else {
					
					// Put a rectangle on the image
					if ((RobotMap.RUNNING_FORWARD && !RobotMap.CAMERAS_SWITCHED) || (!RobotMap.RUNNING_FORWARD && RobotMap.CAMERAS_SWITCHED)) {

						double shift = 3;

						Imgproc.line(matFront, new Point(x1 + shift, 0), new Point(x1 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matFront, new Point(x2 + shift, 0), new Point(x2 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matFront, new Point(x3 + shift, 0), new Point(x3 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matFront, new Point(alignX + shift, 0), new Point(alignX + shift, 120), alignColor, 1);
					}
					// Give the output stream a new image to display
					
					outputStreamFront.putFrame(matFront);
					
				}
			
			
				// ----------------- BACK -----------------------

				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				
				if (cvSinkBack.grabFrame(matBack) == 0) {
					// Send the output the error.
					outputStreamBack.notifyError(cvSinkBack.getError());
				} else {

					if ((!RobotMap.RUNNING_FORWARD && !RobotMap.CAMERAS_SWITCHED) || (RobotMap.RUNNING_FORWARD && RobotMap.CAMERAS_SWITCHED)) {

						double shift = 0;

						Imgproc.line(matBack, new Point(x1 + shift, 0), new Point(x1 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matBack, new Point(x2 + shift, 0), new Point(x2 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matBack, new Point(x3 + shift, 0), new Point(x3 + shift, 120), new Scalar(255, 0, 0), 1);
						Imgproc.line(matBack, new Point(alignX + shift, 0), new Point(alignX + shift, 120), alignColor, 1);
					}

					// Give the output stream a new image to display
					outputStreamBack.putFrame(matBack);
					
				}
					
				
			}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();

		

		// serialPort = new SerialPort(9600, SerialPort.Port.kOnboard);
	}

	/**
	 * Runs once each time the robot is set to disabled
	 */
	@Override
	public void disabledInit() {
		teleopDrive.cancel();

		driveSubsystem.drive(0, 0);
		System.out.println("AFTER DISABLED: " + Devices.getInstance().getNavXGyro().getAngle());
		Scheduler.getInstance().removeAll();

	}

	/**
	 * Runs continuously while the robot is in the disabled state
	 */
	@Override
	public void disabledPeriodic() {
        updateShuffleboardVisualizations();
        jetsonPort.updateVisionValues();
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the autonomous state
	 */
	@Override
	public void autonomousInit() {
		cvLight.set(Value.kForward);
		new ElevatorZero().start();
		teleopDrive = new TeleopDrive();
		teleopDrive.start();
		// new StraightDrive(10, 0.2).start();
	}

	/**
	 * Runs continuously during the autonomous state
	 */
	@Override
	public void autonomousPeriodic() {
        updateShuffleboardVisualizations();
		//System.out.println(jetsonPort.getVisionX1() + " " + jetsonPort.getVisionX2() + " " + jetsonPort.getVisionX3());
		// serialPort.writeString("Hello World!");

		// double voltageMotorSide = Robot.cargo.getSharpVoltageMotorSide();
		// double voltageLimitSide = Robot.cargo.getSharpVoltageLimitSide();
		// System.out.println(voltageMotorSide - voltageLimitSide);

        // System.out.println("Encoder Ticks: " + Robot.elevator.getEncoderTicks());
        jetsonPort.updateVisionValues();

		if (++i % 4 == 0) {
            // jetsonPort.printVisionAngles();
        }

		//System.out.println(jetsonPort.getVisionDistance1());

		// driveSubsystem.printEncoders();

		// System.out.println("Hello World!");
		Scheduler.getInstance().run();
	}

	/**
	 * Runs once before the teleop state
	 */
	@Override
	public void teleopInit() {
		cvLight.set(Value.kForward);
		jetsonPort.updateVisionValues();
		jetsonPort.printVisionAngles();
		teleopDrive = new TeleopDrive();
		teleopDrive.start();
	}

	/**
	 * Runs continuously during the teleop state
	 */
	@Override
	public void teleopPeriodic() {
        updateShuffleboardVisualizations();

		jetsonPort.updateVisionValues();

		//System.out.println(elevator.getEncoderTicks());


		if (++i % 4 == 0) {
			if (RobotMap.CV_RUNNING) {
				//jetsonPort.printVisionAngles();
			}
			//System.out.println("Robot is currently running " + (RobotMap.RUNNING_FORWARD ? "forward." : "backward."));
		}

		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
    }
    
    public void updateShuffleboardVisualizations() {

        RobotMap.CAMERAS_SWITCHED = switchedCameras.getBoolean(false);
        // RobotMap.CAMERAS_SWITCHED = SmartDashboard.getBoolean("Cameras Switched", false);
        // System.out.println(RobotMap.CAMERAS_SWITCHED);

        angle1.setDouble(jetsonPort.getVisionAngle1());
        angle2.setDouble(jetsonPort.getVisionAngle2());
        angle3.setDouble(jetsonPort.getVisionAngle3());
        distance1.setDouble(jetsonPort.getVisionDistance1());
        distance2.setDouble(jetsonPort.getVisionDistance2());
        distance3.setDouble(jetsonPort.getVisionDistance3());

        elevatorPosition.setString(elevator.getExecutingGoalPosition().toString());
        
        if (Robot.cargo.getState() == null) {
            cargoState.setString("Rearming");
        } else {
            cargoState.setString(Robot.cargo.getState().toString());
        }

        cargoSecured.setBoolean(Robot.cargo.getState() == Robot.cargo.CAPTURING);
		
        lastPressedEntry.setString(TeleopDrive.lastPressedPosition.toString());
        
        cvRunning.setBoolean(RobotMap.CV_RUNNING);
        elevatorMoving.setBoolean(elevator.isMoving());
        SmartDashboard.updateValues();
    }

}