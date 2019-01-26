/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

import org.usfirst.frc.team2473.framework.JetsonPort;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
	
	double normalPower = 0.15;
	double turnPower = 0.25;
	private double angle = 0;

	public AlignToHatch() {
		requires(Robot.driveSubsystem);
	}
 
	@Override
	protected void execute() {
		move();
    }
    
    public void move() {
		double thresholdAngle = 3;
        angle = JetsonPort.getInstance().getVisionAngle();
        
		if (Math.abs(angle) < thresholdAngle) { // keep going in this direction
			Robot.driveSubsystem.drive(normalPower, normalPower);
		} else if (Math.abs(angle) > 10) {
<<<<<<< HEAD
			if (angle > thresholdAngle) { // Robot is to the left of the target
				Robot.driveSubsystem.drive(slowPower, -slowPower);
			} else { // Robot is to the right of the target
				Robot.driveSubsystem.drive(-slowPower, slowPower);
			}
		} else {
			if (angle > thresholdAngle) { // Robot is to the left of the target
				Robot.driveSubsystem.drive(slowPower, 0);
			} else { // Robot is to the right of the target
				Robot.driveSubsystem.drive(0, slowPower);
=======
			if (angle > 0) { // Robot is to the left of the target
				Robot.driveSubsystem.drive(turnPower, -turnPower);
			} else { // Robot is to the right of the target
				Robot.driveSubsystem.drive(-turnPower, turnPower);
			}
		} else {
			if (angle > thresholdAngle) { // Robot is to the left of the target
				Robot.driveSubsystem.drive(turnPower, 0);
			} else { // Robot is to the right of the target
				Robot.driveSubsystem.drive(0, turnPower);
>>>>>>> Make CV alignment value receiving quicker
			}
		}
    }

	@Override
	protected boolean isFinished() {
		return false;
	}
}
