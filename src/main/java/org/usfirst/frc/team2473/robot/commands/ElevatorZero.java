/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class ElevatorZero extends Command {

	public ElevatorZero() {
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(-0.2);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return !Robot.oi.getElevatorZero().get();
	}

	@Override
	protected void end() {
		Robot.elevator.stop();
		Robot.elevator.resetEncoders();
		//Robot.elevator.setEncoderZero(Robot.elevator.getEncoderTicks());
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}