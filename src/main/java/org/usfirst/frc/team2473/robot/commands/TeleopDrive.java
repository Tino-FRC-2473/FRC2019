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
 * A class that sets the talons to specific powers upon current joystick positions
 */
public class TeleopDrive extends Command {
	
	private AlignToHatch alignToHatch;

	public TeleopDrive() {
		requires(Robot.driveSubsystem);
	}

	@Override
	protected void initialize() {
		alignToHatch = new AlignToHatch();
	}

	@Override
	protected void execute() {
		double throttleZ = Robot.oi.getThrottle().getZ();
		double wheelX = Robot.oi.getWheel().getX();
		boolean cvButtonPressed = Robot.oi.getCVButton().get();

		if (cvButtonPressed && throttleZ == 0 && wheelX == 0) { // ensures that the CV button is pressed AND the throttle and wheel are zeroed before using CV
			alignToHatch.move();
		} else {
			Robot.driveSubsystem.teleopDrive(throttleZ, wheelX);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
