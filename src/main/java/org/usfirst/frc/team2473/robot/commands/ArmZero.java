package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that resets Arm encoder position
 */
public class ArmZero extends Command {

	public ArmZero() {
		requires(Robot.arm);
		setInterruptible(false);
	}

	@Override
	protected void initialize() {
        Robot.arm.allowZero = true;
        Robot.arm.set(0.1);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return Robot.arm.isUpperLimitSwitchPressed();
	}

	@Override
	protected void end() {
        System.out.println("limit switch pressed");
		Robot.arm.stop();
        Robot.arm.resetEncoders();
        Robot.arm.allowZero = false;
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
