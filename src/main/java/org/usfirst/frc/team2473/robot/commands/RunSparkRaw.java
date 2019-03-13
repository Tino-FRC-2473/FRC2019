package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RunSparkRaw extends Command {

	double power;

	public RunSparkRaw(double power, double seconds) {
		super ("RunSparkRaw", seconds);
		requires(Robot.driveSubsystem);

		this.power = power;
	}

	@Override
	protected void initialize() {
		Robot.driveSubsystem.drive(power, power);
	}

	@Override
	protected void execute() {
        Robot.driveSubsystem.drive(power, power);
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	@Override
	protected void end() {
		Robot.driveSubsystem.stopMotors();
	}

	@Override
	protected void interrupted() {
		Robot.driveSubsystem.stopMotors();
	}
}
