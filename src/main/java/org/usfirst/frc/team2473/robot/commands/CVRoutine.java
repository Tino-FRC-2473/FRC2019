package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * A class for testing a sequence of autonomous commands.
 */
public class CVRoutine extends CommandGroup {

    /**
     * the amount of time to wait in between commands (seconds)
     */
    private final double WAIT = 0.1;

    /**
     * Constructs an empty tester.
     */
    public CVRoutine() {
        addSequential(new AlignToHatch());

        addSequential(new WaitCommand(WAIT));

        addParallel(new ElevatorMove(ElevatorPosition.FIRST, 0.8));
        addParallel(new StraightDrive(20, 0.2));

        addSequential(new WaitCommand(1));

        addSequential(new ElevatorMove(ElevatorPosition.BASE, 0.8));
    }
	
}
