package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class that moves the elevator based on button input
 */
public class ElevatorMoveRaw extends Command {

	private double power;

	public ElevatorMoveRaw(double power) {
		requires(Robot.elevator);

		this.power = power;
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(power);
	}

	@Override
	protected void execute() {
        double tempPower = power;
        if (Robot.elevator.getEncoderTicks() > ElevatorPosition.HATCH_MID.getValue()) {
            if (power > 0) {
                tempPower = 0.3;
            } else {
                tempPower = -0.3;
            }
            
        }
        Robot.elevator.set(tempPower);
	}

	@Override
	protected boolean isFinished() {
		if (power < 0) {
			return !Robot.oi.getElevatorDownButton().get();
		} else {
            //return true;
			return !Robot.oi.getElevatorUpButton().get();
		}
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
