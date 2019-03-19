package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

import java.util.Stack;

import org.usfirst.frc.team2473.framework.State;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Cargo;
import org.usfirst.frc.team2473.robot.subsystems.Arm.ArmPosition;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class enables TeleOp driving for competiton
 */
public class TeleopDrive extends Command {

	private AlignToHatch alignToHatch;

	private final double M = (1 - RobotMap.K_START_STALL_POWER) / (1 - RobotMap.DEADBAND_MINIMUM_POWER);

	double prevAngle;
	double elevatorPower = 0.8;
	double armPower = 0.5;
	
	private Enum lastCargoEvent = null;
	public static ElevatorPosition lastPressedPosition = ElevatorPosition.ZERO;
    
	public static boolean hasRaised = false;
	
	public boolean hasUnlatchedCargoMech = false;

	Stack<Enum> eventStack;

	public TeleopDrive() {
		requires(Robot.driveSubsystem);

		alignToHatch = new AlignToHatch();
		
		eventStack = new Stack<>();
	}

	@Override
	protected void initialize() {
		initializeElevatorButtons();
		initializeArmButtons();
        initializeRawArmButtons();
        Robot.roller.set(0);

        Robot.oi.getCVButton().whenPressed(new InstantCommand() {
            @Override
            protected void execute() {
				if (!RobotMap.CV_RUNNING) return;

				new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.START_CV, elevatorPower, armPower).start();
                hasRaised = false;
            }
        });


		Robot.cargo.setState(Robot.cargo.RELEASING);
	}

	@Override
	protected void execute() {
        // System.out.printf("Elevator: %8.4f     Arm: %8.4f\n", Robot.elevator.getEncoderTicks(), Robot.arm.getEncoderTicks());
        updateDialPosition();
        driveAndAlign();
        // updateCargo();

	}

	public void initializeRawArmButtons() {
		Robot.oi.getElevatorUpButton().whenPressed(new ElevatorMoveRaw(0.4));
		Robot.oi.getElevatorDownButton().whenPressed(new ElevatorMoveRaw(-0.4));

		Robot.oi.getArmUpButton().whenPressed(new ArmMoveRaw(0.2));
		Robot.oi.getArmDownButton().whenPressed(new ArmMoveRaw(-0.2));
	}
	
	public void initializeArmButtons() {
		Robot.oi.getGroundIntakeButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.SCORING_HATCH) return;

				new ElevatorArmMove(ElevatorPosition.CARGO_GROUND, ArmPosition.CARGO_GROUND, elevatorPower, armPower).start();
				Robot.roller.set(-1);
			}
		});

		Robot.oi.getGroundIntakeButton().whenReleased(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.SCORING_HATCH) return;

				ElevatorArmMove e = new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower);
                new ArmRampStow(e).start();
				Robot.roller.set(0);
			}
		});

		Robot.oi.getManualOverrideButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				updateDialPosition();
				new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
			}
		});

		Robot.oi.getManualOverrideButton().whenReleased(new InstantCommand() {
			@Override
			protected void execute() {
				updateDialPosition();
				new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower).start();
			}
		});

		Robot.oi.getReleaseCargoButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.SCORING_HATCH) return;

				Robot.roller.set(1);
			}
		});

		Robot.oi.getReleaseCargoButton().whenReleased(new InstantCommand() {
			@Override
			protected void execute() {
				if (RobotMap.SCORING_HATCH) return;

				Robot.roller.set(0);
			}
		});
	}

	public static ArmPosition getArmPositionFromElevator() {
		ArmPosition armPos = null;
		switch (lastPressedPosition) {
			case ZERO:
				armPos = ArmPosition.ZERO;
				break;
			case HATCH_PICKUP:
				armPos = ArmPosition.HATCH_PICKUP;
				break;
			case CARGO_PICKUP:
				armPos = ArmPosition.CARGO_PICKUP;
				break;
			case HATCH_LOW:
				armPos = ArmPosition.HATCH_LOW;
				break;
			case CARGO_LOW:
				armPos = ArmPosition.CARGO_LOW;
				break;
			case HATCH_MID:
				armPos = ArmPosition.HATCH_MID;
				break;
			case CARGO_MID:
				armPos = ArmPosition.CARGO_MID;
				break;
			case HATCH_HIGH:
				armPos = ArmPosition.HATCH_HIGH;
				break;
			case CARGO_HIGH:
				armPos = ArmPosition.CARGO_HIGH;
				break;
		}
		return armPos;
	}
    
    public void initializeElevatorButtons() {

        /*
		-------------------------------------
		 E L E V A T O R   M E C H A N I S M 
		-------------------------------------
		*/

		Robot.oi.getElevatorPickupButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				updateDialPosition();
				
				if (!hasUnlatchedCargoMech) return;

				if (!RobotMap.CV_RUNNING) {
                    new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
                } else {
                    new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower).start();
                }
			}
		});

		Robot.oi.getElevatorLowButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				updateDialPosition();
				
				if (!hasUnlatchedCargoMech) return;

                if (!RobotMap.CV_RUNNING) {
                    new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
                } else {
                    new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower).start();
                }
			}
		});

		Robot.oi.getElevatorMidButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				updateDialPosition();
				
				if (!hasUnlatchedCargoMech) return;
				
                if (!RobotMap.CV_RUNNING) {
                    new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
                } else {
                    new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower).start();
                }
                
            }
		});

        Robot.oi.getElevatorHighButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
                updateDialPosition();
				
				if (!hasUnlatchedCargoMech) return;
				
                if (!RobotMap.CV_RUNNING) {
                    new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
                } else {
                    new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower).start();
                }
			}
		});

		Robot.oi.getReleaseElementButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				if (!hasUnlatchedCargoMech) {
					new ElevatorMove(ElevatorPosition.RELEASE_CARGO_MECH, false, elevatorPower).start();
					hasUnlatchedCargoMech = true;
				} else if (RobotMap.CV_RUNNING && RobotMap.SCORING_HATCH) {
					new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), true, elevatorPower).start();
				}
			}
		});

		Robot.oi.getReleaseElementButton().whenReleased(new InstantCommand() {
			@Override
			protected void execute() {
				if (hasUnlatchedCargoMech && RobotMap.CV_RUNNING && RobotMap.SCORING_HATCH) {
					new ElevatorMove(Robot.elevator.getExecutingGoalPosition(), false, elevatorPower).start();
				}
			}
		});
    }
    
    public void updateDialPosition() {
        boolean pickupPosition = Robot.oi.getElevatorPickupButton().get();
        boolean lowPosition = Robot.oi.getElevatorLowButton().get();
        boolean midPosition = Robot.oi.getElevatorMidButton().get();
        boolean highPosition = Robot.oi.getElevatorHighButton().get();

        if (pickupPosition) {
            lastPressedPosition = (RobotMap.SCORING_HATCH) ? ElevatorPosition.HATCH_PICKUP : ElevatorPosition.CARGO_PICKUP;
        } else if (lowPosition) {
            lastPressedPosition = (RobotMap.SCORING_HATCH) ? ElevatorPosition.HATCH_LOW : ElevatorPosition.CARGO_LOW;
        } else if (midPosition) {
            lastPressedPosition = (RobotMap.SCORING_HATCH) ? ElevatorPosition.HATCH_MID : ElevatorPosition.CARGO_MID;
        } else if (highPosition) {
            lastPressedPosition = (RobotMap.SCORING_HATCH) ? ElevatorPosition.HATCH_HIGH : ElevatorPosition.CARGO_HIGH;
        } else {
            if (lastPressedPosition != null) {
                ElevatorArmMove e = new ElevatorArmMove(ElevatorPosition.HATCH_LOW, ArmPosition.STOW, elevatorPower, armPower);
                new ArmRampStow(e).start();
                lastPressedPosition = null;
            }
            
        }
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

		if (RobotMap.CV_RUNNING && Robot.oi.getCVButton().get() && lastPressedPosition != null) {
            alignToHatch.move();
            // if (!hasMovedArm) {
            //     new ArmMove(ArmPosition.START_CV, armPower).start();
            //     hasMovedArm = true;
			// }
			
            // if (!RobotMap.MANUAL_CONTROL) {
            //     int highRaiseDist = 60;
            //     if (!RobotMap.SCORING_HATCH) highRaiseDist = 50;
            //     if (AlignToHatch.isRunning && alignToHatch.distance < highRaiseDist && !hasRaised) {
            //         System.out.println("HIGH RAISE");
            //         new ElevatorArmMove(lastPressedPosition, getArmPositionFromElevator(), elevatorPower, armPower).start();
            //         hasRaised = true;
            //     }
            // }
            
		} else { // Move using controls, not CV
            alignToHatch.reset();
            // if (RobotMap.SCORING_HATCH && AlignToHatch.isRunning != wasRunningCV) {
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
			} else {
				if (wheelX != 0) {
					outputX = 0.1;
				} else {
					outputX = 0;
				}
			}

			outputX *= 1.5;
			if (outputX > 1) {
				outputX = 1;
			} else if (outputX < -1) {
				outputX = -1;
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

		if (Robot.oi.getGroundIntakeButton().get()) {
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
