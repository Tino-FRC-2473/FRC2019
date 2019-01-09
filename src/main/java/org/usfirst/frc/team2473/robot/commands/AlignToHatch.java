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

import org.usfirst.frc.team2473.framework.UtilitySocket;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
		
	double theta;
	
	double normalPower = 0.25;
	double addedPower = 0.15;
		
	private UtilitySocket s;
		
	public AlignToHatch() {
		requires(Robot.driveSubsystem);
		
		try {
			System.out.println("Creating utility socket in AlignToHatch");
			s = new UtilitySocket("10.24.73.19", 5801);
			System.out.println("Utility socket created!");
		} catch (IOException e) {
			System.out.println("Error in Constructor try-catch");
			e.printStackTrace();
		}
	}
	
	private void updateTheta() {
		String thetaStr = s.getLine();
		if (thetaStr == null) {
			System.out.println("No new value of theta");
		} else {
			theta = Double.parseDouble(thetaStr);
		}
		
	}
	
	@Override
	protected void initialize() {
		updateTheta();
	}
 
	@Override
	protected void execute() {
		updateTheta();
		System.out.println(theta);
		if (Math.abs(theta) < 1) { // keep going in this direction
			Robot.driveSubsystem.drive(normalPower, normalPower, normalPower, normalPower);
		} else if (theta > 1) { // Robot is to the left of the target
			Robot.driveSubsystem.drive(normalPower + addedPower, normalPower + addedPower, normalPower, normalPower);
		} else { // Robot is to the right of the target
			Robot.driveSubsystem.drive(normalPower, normalPower, normalPower + addedPower, normalPower + addedPower);
		}
		
		/*
		This could end up weaving left and right. If we could determine when we are head on with the tape,
		then we could conditions 2 and 3 above until theta == 0. Then, we turn until we are head-on with the
		tape. Finally, we continue straight until we reach the hatch.
		*/
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
