package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that test encoder ticks with the elevator
 */
public class ElevatorTicksTest extends Command {

	public ElevatorTicksTest() {
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(0.2);
	}

	@Override
	protected void execute() {
		Robot.elevator.printEncoders();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.elevator.stop();
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
