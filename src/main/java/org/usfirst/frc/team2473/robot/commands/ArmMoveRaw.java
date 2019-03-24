package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that moves the arm based on button imput
 */
public class ArmMoveRaw extends Command {

	private double power;

	public ArmMoveRaw(double power) {
		requires(Robot.arm);

		this.power = power;
	}

	@Override
	protected void initialize() {
		Robot.arm.set(power);
	}

	@Override
	protected void execute() {
        double tempPower = power;
        Robot.arm.set(tempPower);
	}

	@Override
	protected boolean isFinished() {
		if (power < 0) {
			return !Robot.oi.getArmDownButton().get();
		} else {
            //return true;
			return !Robot.oi.getArmUpButton().get();
		}
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
