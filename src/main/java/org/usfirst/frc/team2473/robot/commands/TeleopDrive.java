/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

import java.util.Stack;

import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.framework.State;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Cargo;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class that sets the talons to specific powers upon current joystick
 * positions
 */
public class TeleopDrive extends Command {

	private AlignToHatch alignToHatch;

	private final double M = (1 - RobotMap.K_START_STALL_POWER) / (1 - RobotMap.DEADBAND_MINIMUM_POWER);

	double prevAngle;
	double power = 0.8;
	
    private Enum lastCargoEvent = null;
    
    private boolean hasLoweredElevator = false;

	Stack<Enum> eventStack;

	public TeleopDrive() {
		requires(Robot.driveSubsystem);

		alignToHatch = new AlignToHatch();
		
		eventStack = new Stack<>();
	}

	@Override
	protected void initialize() {
        prevAngle = Robot.jetsonPort.getVisionAngle1();

		/*
		-------------------------------------
		 E L E V A T O R   M E C H A N I S M 
		-------------------------------------
		*/

		Robot.oi.getElevatorZeroButton().whenPressed(new ElevatorZero());

		Robot.oi.getElevatorPickupButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(ElevatorPosition.HATCH_PICKUP, false, power).start();
				} else {
					new ElevatorMove(ElevatorPosition.CARGO_PICKUP, false, power).start();
				}
			}
		});

		Robot.oi.getElevatorLowButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(ElevatorPosition.HATCH_LOW, false, power).start();
				} else {
					new ElevatorMove(ElevatorPosition.CARGO_LOW, false, power).start();
				}
			}
		});

		Robot.oi.getElevatorMidButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(ElevatorPosition.HATCH_MID, false, power).start();
				} else {
					new ElevatorMove(ElevatorPosition.CARGO_MID, false, power).start();
				}
			}
		});

        Robot.oi.getElevatorHighButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(ElevatorPosition.HATCH_HIGH, false, power).start();
				} else {
					new ElevatorMove(ElevatorPosition.CARGO_HIGH, false, power).start();
				}
			}
		});
        
        

        Robot.oi.getElevatorInitialStowButton().whenPressed(new ElevatorMoveRaw(power/2));
        Robot.oi.getElevatorDown().whenPressed(new ElevatorMoveRaw(-power/2));


		Robot.cargo.setState(Robot.cargo.RELEASING);
	}

	@Override
	protected void execute() {
        // Robot.elevator.printEncoders();
		double throttleZ = Robot.oi.getThrottle().getZ();
		double originalZ = throttleZ;
		double wheelX = -Robot.oi.getWheel().getX();

		double outputZ = 0;
		double outputX = 0;

		//System.out.println(throttleZ + " " + wheelX);

		/* Scale throttle values to:

		DEADBAND_MINIMUM_POWER ==> K_START_STALL_POWER
		1 ==> 1

		*/

		throttleZ = M*(throttleZ - RobotMap.DEADBAND_MINIMUM_POWER);

		//System.out.println("scaled " + throttleZ + " " + wheelX);


        // Align To Hatch
        

        boolean wasRunningCV = AlignToHatch.isRunning;

		if (RobotMap.CV_RUNNING && Robot.oi.getCVButton().get()) {
            alignToHatch.move();
            if (RobotMap.RUNNING_FORWARD && AlignToHatch.isRunning != wasRunningCV) {
                // we are starting CV
                new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), false, power).start();
            }
		} else { // Move using controls, not CV
            alignToHatch.reset();
            
            if (RobotMap.RUNNING_FORWARD && AlignToHatch.isRunning != wasRunningCV) {
                // we are stopping CV
                new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), true, power).start();
            }

			alignToHatch.calculateTarget(); // for x value
			// Deadband
			if (Math.abs(throttleZ) > RobotMap.DEADBAND_MINIMUM_POWER) {
				outputZ = throttleZ;
			}

			if (Math.abs(wheelX) > RobotMap.DEADBAND_MINIMUM_TURN) {
				outputX = wheelX;
			}

			Robot.driveSubsystem.teleopDrive(outputZ, outputX);
		}

		/*
		-------------------------------
		 C A R G O   M E C H A N I S M 
		-------------------------------
		*/

		Enum event = null;

		double voltageMotorSide = Robot.cargo.getSharpVoltageMotorSide();
		double voltageLimitSide = Robot.cargo.getSharpVoltageLimitSide();

		//System.out.printf("%.5f %.5f\n", voltageMotorSide, voltageLimitSide);

		if (voltageMotorSide < Cargo.UNSAFE_VOLTAGE_MIN && voltageLimitSide < Cargo.UNSAFE_VOLTAGE_MIN) { // not holding a ball
			event = Cargo.BallEvent.NONE;
		} else if ((voltageMotorSide >= Cargo.UNSAFE_VOLTAGE_MIN && voltageMotorSide <= Cargo.UNSAFE_VOLTAGE_MAX)
				|| (voltageLimitSide >= Cargo.UNSAFE_VOLTAGE_MIN && voltageLimitSide <= Cargo.UNSAFE_VOLTAGE_MAX)
				|| (Math.abs(voltageMotorSide - voltageLimitSide) >= 0.5)) {
			event = Cargo.BallEvent.UNSAFE;
		} else if (voltageMotorSide >= Cargo.CAPTURE_VOLTAGE) {
			event = Cargo.BallEvent.CAPTURED;
		} else {
			event = Cargo.BallEvent.SAFE;
		}

		if (lastCargoEvent != event) {
			eventStack.add(event);
			System.out.println("ADDING " + event + " TO STACK");
			lastCargoEvent = event;
		}

		if (Robot.oi.getCargoButton().get()) {
			eventStack.add(Cargo.RequestEvent.RELEASE);
		}

		while (!eventStack.isEmpty()) {
			State newState = Robot.cargo.getState().handleEvent(eventStack.pop());
			if (newState != null) {
				System.out.println("CHANGING STATE TO " + newState + " ----------------------------------");
				Robot.cargo.setState(newState);
			}
		}

	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
