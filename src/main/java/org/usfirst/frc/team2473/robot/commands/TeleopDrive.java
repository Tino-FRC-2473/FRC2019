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
import org.usfirst.frc.team2473.robot.subsystems.Elevator;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class that sets the talons to specific powers upon current joystick
 * positions
 */
public class TeleopDrive extends Command {

	private AlignToHatch alignToHatch;

	private final double M = (1 - RobotMap.K_START_STALL_POWER) / (1 - RobotMap.DEADBAND_MINIMUM_POWER);

	double prevAngle;
	double power = 0.6;
	
	private Enum lastCargoEvent = null;
	public static ElevatorPosition lastPressedPosition = ElevatorPosition.ZERO;
    
    private boolean hasIncreasedElevatorHeight = false;

	Stack<Enum> eventStack;

	public TeleopDrive() {
		requires(Robot.driveSubsystem);

		alignToHatch = new AlignToHatch();
		
		eventStack = new Stack<>();
	}

	@Override
	protected void initialize() {
        initializeElevatorButtons();

        Robot.oi.getCVButton().whenPressed(new InstantCommand() {
            @Override
            protected void execute() {
                if (!isLastPressedCargo() && !(lastPressedPosition == ElevatorPosition.HATCH_HIGH)) {
                    new ElevatorMove(lastPressedPosition, false, power).start();
                } else if (isLastPressedCargo()) {
                    new ElevatorMove(ElevatorPosition.CARGO_PICKUP, false, power).start();
                } else {
                    if (Robot.elevator.getExecutingGoalPosition() != ElevatorPosition.HATCH_HIGH) {
                        new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), false, power).start();
                    }
                    else {
                        new ElevatorMove(ElevatorPosition.HATCH_LOW, false, power).start();
                    }
                }
                hasIncreasedElevatorHeight = false;
            }
        });


		Robot.cargo.setState(Robot.cargo.RELEASING);
	}

	@Override
	protected void execute() {
		
        driveAndAlign();
		// updateCargo();

    }

    public void initializeElevatorButtons() {

        /*
		-------------------------------------
		 E L E V A T O R   M E C H A N I S M 
		-------------------------------------
		*/


        Robot.oi.getElevatorZeroButton().whenPressed(new ElevatorZero());

		Robot.oi.getElevatorPickupButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
                lastPressedPosition = RobotMap.RUNNING_FORWARD ? ElevatorPosition.HATCH_PICKUP : ElevatorPosition.CARGO_PICKUP;
				//new ElevatorMove(lastPressedPosition, false, power).start();
			}
		});

		Robot.oi.getElevatorLowButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				lastPressedPosition = RobotMap.RUNNING_FORWARD ? ElevatorPosition.HATCH_LOW : ElevatorPosition.CARGO_LOW;
				//new ElevatorMove(lastPressedPosition, false, power).start();
			}
		});

		Robot.oi.getElevatorMidButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				lastPressedPosition = RobotMap.RUNNING_FORWARD ? ElevatorPosition.HATCH_MID : ElevatorPosition.CARGO_MID;
				//new ElevatorMove(lastPressedPosition, false, power).start();
			}
		});

        Robot.oi.getElevatorHighButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				lastPressedPosition = RobotMap.RUNNING_FORWARD ? ElevatorPosition.HATCH_HIGH : ElevatorPosition.CARGO_HIGH;
				//new ElevatorMove(lastPressedPosition, false, power).start();
			}
		});

		Robot.oi.getReleaseElementButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.CV_RUNNING && RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), true, power).start();
				}
			}
		});

		Robot.oi.getReleaseElementButton().whenReleased(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.CV_RUNNING && RobotMap.RUNNING_FORWARD) {
					new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), false, power).start();
				}
			}
		});
        
        Robot.oi.getElevatorInitialStowButton().whenPressed(new ElevatorMoveRaw(0.5));
        Robot.oi.getElevatorDown().whenPressed(new ElevatorMoveRaw(-0.5));
    }
    
    public void driveAndAlign() {
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

		if (RobotMap.CV_RUNNING && Robot.oi.getCVButton().get()) {
            alignToHatch.move();

            int highRaiseDist = 30;
			if ((isLastPressedCargo() || lastPressedPosition == ElevatorPosition.HATCH_HIGH) && !hasIncreasedElevatorHeight && AlignToHatch.isRunning && alignToHatch.distance < highRaiseDist) {
				new ElevatorMove(lastPressedPosition, false, power).start();
				hasIncreasedElevatorHeight = true;
			}
		} else { // Move using controls, not CV
            alignToHatch.reset();
            // if (RobotMap.RUNNING_FORWARD && AlignToHatch.isRunning != wasRunningCV) {
            //     // we are stopping CV
            //     new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), true, power).start();
            // }

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
    }

    public void updateCargo() {
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
				|| (Math.abs(voltageMotorSide - voltageLimitSide) >= 0.6)) {
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

    public boolean isLastPressedCargo() {
        return lastPressedPosition == ElevatorPosition.CARGO_PICKUP || lastPressedPosition == ElevatorPosition.CARGO_LOW || lastPressedPosition == ElevatorPosition.CARGO_MID || lastPressedPosition == ElevatorPosition.CARGO_HIGH;
    }
}
