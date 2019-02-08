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
public class ElevatorTicksTest extends Command {

	public ElevatorTicksTest() {
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(0.2);
	}

	@Override
	protected void execute() {
		Robot.elevator.printEncoders();
	}

	@Override
	protected boolean isFinished() {
		return false;
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
