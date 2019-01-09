/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A class that enables straight driving of the robot to a given distance
 * at a given power.
 */
public class StraightDrive extends Command {
	
	/**
	 * Lowest power robot will run at towards the end of
	 * the straight drive.
	 */
	private static double SLOW_POWER = 0.1;
	
	/**
	 * Number of inches to move by.
	 */
	private double moveByInches;
	
	/**
	 * The number of ticks that the robot should
	 * be at if it has reached its goal.
	 */
	private double absoluteTickGoal;
	
	/**
	 * The absolute number of ticks that the robot
	 * was at after the last call of execute.
	 */
	private int prevTicks;
	
	/**
	 * Initial power to drive the robot at.
	 */
	private double power;
	
	/**
	 * Create a StraightDrive object with given inch goal and power.
	 * @param inches inches to move by
	 * @param power initial power to move at
	 */
	public StraightDrive(double inches, double power) {
		requires(Robot.driveSubsystem);
		
		this.moveByInches = inches;
		this.power = (inches < 0) ? -power : power;
	}
	
	/**
	 * Set the power to move the robot at.
	 * @param power power
	 */
	public void setPower(double power) {
		this.power = power;
	}
	
	/**
	 * Set the target distance
	 */
	private void setDistance(double inches) {
		System.out.println("TICKSSSSS: " + Robot.driveSubsystem.getEncoderTicks(RobotMap.TALON_FR));
		this.absoluteTickGoal = Robot.driveSubsystem.getEncoderTicks(RobotMap.TALON_FR) + (this.moveByInches * RobotMap.K_TICKS_PER_INCH);
	}

	@Override
	protected void initialize() {
		setDistance(moveByInches);
		prevTicks = Robot.driveSubsystem.getEncoderTicks(RobotMap.TALON_FR);
		
		System.out.println("ANGLE: " + Robot.driveSubsystem.getGyroAngle());

		System.out.println("REQUIRED TICKS: " + absoluteTickGoal);
		Robot.driveSubsystem.drive(power, power, power, power);
	}

	@Override
	protected void execute() {
		double tempPower = power;
		int currTicks = Robot.driveSubsystem.getEncoderTicks(RobotMap.TALON_FR);
		
		int delta = currTicks - prevTicks;
		
		/* If the robot has exceeded the threshold below, it will move at a slower power */
		if (Math.abs(absoluteTickGoal - (currTicks + delta)) < RobotMap.K_ENCODER_THRESHOLD) { // Math.abs() allows this to work regardless of driving direction (forwards or backwards)
			if (moveByInches > 0) tempPower = SLOW_POWER;
			else tempPower = -SLOW_POWER;
		}
		Robot.driveSubsystem.drive(tempPower,tempPower,tempPower,tempPower);
				
		prevTicks = currTicks;
		
		Robot.driveSubsystem.printEncoders();
		
	}

	@Override
	protected boolean isFinished() {
		int currTicks = Robot.driveSubsystem.getEncoderTicks(RobotMap.TALON_FR);
		if (this.moveByInches > 0) return (absoluteTickGoal < currTicks);
		else return (absoluteTickGoal > currTicks);
	}

	@Override
	protected void end() {
		System.out.println(power);
		System.out.println("----------------");
		System.out.println("REQUIRED TICKS: " + absoluteTickGoal);
		Robot.driveSubsystem.printEncoders();		
		System.out.println("Difference: " + Robot.driveSubsystem.encoderDifference());
		
		System.out.println();
		
		Robot.driveSubsystem.stopMotors();
	}

	@Override
	protected void interrupted() {
		Robot.driveSubsystem.stopMotors();
	}
}