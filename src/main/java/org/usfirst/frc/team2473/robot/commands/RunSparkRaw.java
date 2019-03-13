package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class RunSparkRaw extends Command {

	double power;

	public RunSparkRaw(double power, double seconds) {
		super ("RunSparkRaw", seconds);
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveSubsystem);

		this.power = power;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveSubsystem.drive(power, power);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        Robot.driveSubsystem.drive(power, power);
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
		Robot.driveSubsystem.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.driveSubsystem.stopMotors();
	}
}
