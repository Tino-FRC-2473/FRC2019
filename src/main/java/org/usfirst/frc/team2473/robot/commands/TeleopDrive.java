/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

import org.usfirst.frc.team2473.framework.JetsonPort;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

/**
 * A class that sets the talons to specific powers upon current joystick
 * positions
 */
public class TeleopDrive extends Command {

	private AlignToHatch alignToHatch;

	private final double M = (1 - RobotMap.K_START_STALL_POWER) / (1 - RobotMap.DEADBAND_MINIMUM_POWER);

	double prevAngle;

	public TeleopDrive() {
		requires(Robot.driveSubsystem);

		alignToHatch = new AlignToHatch();
	}

	@Override
	protected void initialize() {
        prevAngle = Robot.jetsonPort.getVisionAngle();
        
        Robot.oi.getReverseDriveButton().whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				RobotMap.RUNNING_FORWARD = !RobotMap.RUNNING_FORWARD;
			}
		});
	}

	@Override
	protected void execute() {

		double throttleZ = Robot.oi.getThrottle().getZ();
		double originalZ = throttleZ;
		double wheelX = -Robot.oi.getWheel().getX();

		double outputZ = 0;
		double outputX = 0;

		//System.out.println(throttleZ + " " + wheelX);

		/* Scale throttle values to:

		DEADBAND_MINIMUM_POWER ==> K_START_STALL_POWER
		1 ==> 1

		*/

		throttleZ = M*(throttleZ - RobotMap.DEADBAND_MINIMUM_POWER);

		//System.out.println("scaled " + throttleZ + " " + wheelX);

		// Align To Hatch
		if (RobotMap.CV_RUNNING && Robot.oi.getCVButton().get() && Math.abs(originalZ) < RobotMap.DEADBAND_MINIMUM_POWER && Math.abs(wheelX) < RobotMap.DEADBAND_MINIMUM_TURN) {
			alignToHatch.move();
		} else { // Move using controls, not CV

			// Deadband
			if (Math.abs(throttleZ) > RobotMap.DEADBAND_MINIMUM_POWER) {
				outputZ = throttleZ;
			}

			if (Math.abs(wheelX) > RobotMap.DEADBAND_MINIMUM_TURN) {
				outputX = wheelX;
			}

			Robot.driveSubsystem.teleopDrive(outputZ, outputX);
		}

	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
