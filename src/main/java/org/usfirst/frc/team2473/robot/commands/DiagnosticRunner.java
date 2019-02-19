package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * A class for testing a sequence of autonomous commands.
 */
public class DiagnosticRunner extends Command {
	
	/**
	 * the amount of time to wait in between commands (seconds)
	 */
    private int currentCommand = 0;
	private final double WAIT = 1;
    private static final Command[] COMMANDS = {
        new ElevatorZero(),

        new Command(3, Robot.driveSubsystem){
            @Override
            protected void execute() {
                Robot.driveSubsystem.drive(0.5, 0.5);
            }
            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }
        },
        new Command(3, Robot.driveSubsystem){
            @Override
            protected void execute() {
                Robot.driveSubsystem.drive(-0.5, -0.5);
            }
            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }
        },
        new Command(3, Robot.driveSubsystem){
            @Override
            protected void execute() {
                Robot.driveSubsystem.drive(-0.5, 0.5);
            }
            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }
        },
        new Command(3, Robot.driveSubsystem){
            @Override
            protected void execute() {
                Robot.driveSubsystem.drive(0.5, -0.5);
            }
            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }
        },

        new ElevatorMove(ElevatorPosition.HATCH_PICKUP, false, 1),
        new ElevatorMove(ElevatorPosition.HATCH_PICKUP, true, 1),
        new ElevatorMove(ElevatorPosition.HATCH_LOW, false, 1),
        new ElevatorMove(ElevatorPosition.HATCH_LOW, true, 1),
        new ElevatorMove(ElevatorPosition.HATCH_MID, false, 1),
        new ElevatorMove(ElevatorPosition.HATCH_MID, true, 1),
        new ElevatorMove(ElevatorPosition.HATCH_HIGH, false, 1),
        new ElevatorMove(ElevatorPosition.HATCH_HIGH, true, 1),

        new ElevatorZero(),

        new ElevatorMove(ElevatorPosition.CARGO_PICKUP, false, 1),
        new ElevatorMove(ElevatorPosition.CARGO_LOW, false, 1),
        new ElevatorMove(ElevatorPosition.CARGO_MID, false, 1),
        new ElevatorMove(ElevatorPosition.CARGO_HIGH, false, 1),

        new ElevatorZero()
    };
	/**
	 * Constructs an empty tester.
	 */
	public DiagnosticRunner() {
        
    }

    @Override
    protected void initialize() {
        Robot.oi.getCVButton().whenPressed(new InstantCommand() {
            @Override
            protected void execute() {
                COMMANDS[currentCommand].start();
                currentCommand++;
                if (currentCommand == COMMANDS.length) {
                    currentCommand = 0;
                    RobotMap.CV_RUNNING = false;
                }
            }
        });
        super.initialize();
    }

    @Override
    protected void execute() {

        super.execute();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }


}
