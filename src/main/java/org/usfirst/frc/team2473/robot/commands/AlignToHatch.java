/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

import java.io.IOException;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.UtilitySocket;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
	
	double normalPower = 0.25;
    double addedPower = 0.15;
    		
	public AlignToHatch() {
		requires(Robot.driveSubsystem);
	}
	
	@Override
	protected void initialize() {
		Robot.driveSubsystem.drive(normalPower, normalPower, normalPower + addedPower, normalPower + addedPower);
	}
 
	@Override
	protected void execute() {
		move();
    }
    
    public void move() {
        double angle = Devices.getInstance().getCVAngle();
		System.out.println(angle);
		if (Math.abs(angle) < 1) { // keep going in this direction
			Robot.driveSubsystem.drive(normalPower, normalPower, normalPower, normalPower);
		} else if (angle > 1) { // Robot is to the left of the target
			Robot.driveSubsystem.drive(normalPower + addedPower, normalPower + addedPower, normalPower, normalPower);
		} else { // Robot is to the right of the target
			Robot.driveSubsystem.drive(normalPower, normalPower, normalPower + addedPower, normalPower + addedPower);
		}
    }

	@Override
	protected boolean isFinished() {
		return false;
	}
}
