package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that smoothly transitions the power of the arm to remove jerk
 */
public class ArmRampDown extends Command {

	public ArmRampDown() {
		requires(Robot.arm);
	}

	@Override
	protected void initialize() {
        Robot.arm.set(Robot.arm.lastNonZeroPower);
        System.out.println("RAMP: "+Robot.arm.getPower());

	}

	@Override
	protected void execute() {
        System.out.println("RAMP: "+Robot.arm.getPower());
        if (Robot.arm.getPower() < 0) {
            Robot.arm.set(Math.min(0, Robot.arm.getPower() + 0.02));
        } else {
            Robot.arm.set(Math.max(0, Robot.arm.getPower() - 0.02));
        }
	}

	@Override
	protected boolean isFinished() {
        return Robot.arm.getPower() == 0;
	}

	@Override
	protected void end() {
		Robot.arm.stop();
	}

	@Override
	protected void interrupted() {
		Robot.arm.stop();
	}
}
