package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class RunRoller extends Command {

	double power;

	public RunRoller(double power, double seconds) {
		super ("RunRoller", seconds);
		// Use requires() here to declare subsystem dependencies
		requires(Robot.roller);

		this.power = power;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.roller.set(power);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//Robot.sparkSubsystem.printEncoderInfo();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.roller.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.roller.set(0);
	}
}
