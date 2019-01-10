/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

import org.usfirst.frc.team2473.framework.Devices;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
	
	private double normalPower = 0.25;
    private double addedPower = 0.15;
    		
	public AlignToHatch() {
		requires(Robot.driveSubsystem);
	}
 
	@Override
	protected void execute() {
		move();
    }
    
    public void move() {
        double angle = Devices.getInstance().getCVAngle();
        double distance = Devices.getInstance().getCVDistance();

        double adjustAddedPower = addedPower;
        double adjustNormalPower = normalPower;
        if (distance < 10 && Math.abs(angle) > 10) {
            adjustAddedPower *= 1.5;
            adjustNormalPower = 0.1;
        }

		System.out.println(angle);
		if (Math.abs(angle) < 1) { // keep going in this direction
			Robot.driveSubsystem.drive(adjustNormalPower, adjustNormalPower, adjustNormalPower, adjustNormalPower);
		} else if (angle > 1) { // Robot is to the left of the target
			Robot.driveSubsystem.drive(adjustNormalPower + adjustAddedPower, adjustNormalPower + adjustAddedPower, adjustNormalPower, adjustNormalPower);
		} else { // Robot is to the right of the target
			Robot.driveSubsystem.drive(adjustNormalPower, adjustNormalPower, adjustNormalPower + adjustAddedPower, adjustNormalPower + adjustAddedPower);
		}
    }

	@Override
	protected boolean isFinished() {
		return Devices.getInstance().getCVDistance() < 5;
	}
}