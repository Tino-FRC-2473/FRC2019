package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator;

/**
 * A class resets encoders on the elevator
 */
public class ElevatorZero extends Command {

	public ElevatorZero() {
		requires(Robot.elevator);
		setInterruptible(false);
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(-0.1);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return Robot.elevator.isLowerLimitSwitchPressed();
	}

	@Override
	protected void end() {
        System.out.println("limit switch pressed");
		Robot.elevator.stop();
		Robot.elevator.resetEncoders();
		Robot.elevator.setExecutingGoalPosition(Elevator.ElevatorPosition.ZERO);
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
