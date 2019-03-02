/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
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
