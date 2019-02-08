/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.framework.JetsonPort;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
	
	double normalPower = 0.2;
	double turnPower = 0.1;
	private double angle = 0;
    private double distance = 0;

	public AlignToHatch() {
		//requires(Robot.driveSubsystem);
	}
 
	@Override
	protected void execute() {
		move();
    }
	
	//this method is used in TeleopDrive rather than execute
    public void move() {
		updateSmartDashboard(); //put thit here since execute is not always called

		if (!RobotMap.CV_RUNNING) return;
        
		double thresholdAngle = 3;
        /*angle = Robot.jetsonPort.getVisionAngle();
        distance = Robot.jetsonPort.getVisionDistance();
        if (!RobotMap.RUNNING_FORWARD) angle = -angle;
        if (distance < 20) {
            Robot.driveSubsystem.drive(0, 0);
		} else if (Math.abs(angle) < thresholdAngle) { // keep going in this direction
			Robot.driveSubsystem.drive(normalPower, normalPower);
		} else if (Math.abs(angle) > 10) {
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
			}
		}*/
    }

	public void updateSmartDashboard(){
		SmartDashboard.putNumber("Angle", Robot.jetsonPort.getVisionAngle());
		SmartDashboard.putNumber("Distance", Robot.jetsonPort.getVisionDistance());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		SmartDashboard.putNumber("Angle", -90);
		super.end();
		SmartDashboard.putNumber("Distance", -1000);
	}
}
