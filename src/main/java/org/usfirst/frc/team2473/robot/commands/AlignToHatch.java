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
	double slowPower = 0.07;
    double addedPower = 0.15;
	private int numMoves = 0;
	private double angle = 0;

	public AlignToHatch() {
		requires(Robot.driveSubsystem);
	}
 
	@Override
	protected void execute() {
		move();
    }
    
    public void move() {
		numMoves++;
		double thresholdAngle = 3;

		//if (numMoves % 3 == 0) {
		angle = JetsonPort.getInstance().getVisionAngle();
		//}
        
		//System.out.println(angle);
		

		
		
		if (Math.abs(angle) < thresholdAngle) { // keep going in this direction
			Robot.driveSubsystem.drive(normalPower, normalPower);
		} else if (Math.abs(angle) > 10) {
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
			}
		}

		// if (Math.abs(angle) < 1) {
		// 	Robot.driveSubsystem.drive(normalPower, normalPower, normalPower, normalPower);
		// } else {
		// 	PointTurn p = new PointTurn(angle, 0.45);
		// 	p.initialize();
		// 	p.move();
		// }
    }

	@Override
	protected boolean isFinished() {
		return false;
	}
}
