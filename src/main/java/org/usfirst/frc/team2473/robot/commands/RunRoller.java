package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A class that can run the roller on a command
 */
public class RunRoller extends Command {

	double power;

	public RunRoller(double power, double seconds) {
		super ("RunRoller", seconds);
		requires(Robot.roller);

		this.power = power;
	}

	@Override
	protected void initialize() {
		Robot.roller.set(power);
	}

	@Override
	protected void execute() {}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	@Override
	protected void end() {
		Robot.roller.set(0);
	}

	@Override
	protected void interrupted() {
		Robot.roller.set(0);
	}
}
