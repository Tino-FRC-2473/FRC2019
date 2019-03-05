/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class ArmZero extends Command {

	public ArmZero() {
		requires(Robot.arm);
		setInterruptible(false);
	}

	@Override
	protected void initialize() {
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
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
