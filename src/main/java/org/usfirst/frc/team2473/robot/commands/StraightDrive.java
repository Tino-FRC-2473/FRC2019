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
	private double prevTicks;
	
	/**
	 * Initial power to drive the robot at.
	 */
	private double power;
	
	/**
	 * Create a StraightDrive object with given inch goal and power.
	 * @param inches inches to move by (can be negative)
	 * @param power initial power to move at (must be positive)
	 */
	public StraightDrive(double inches, double power) {
		if(power < 0) throw new IllegalArgumentException("Power must be positive!");

		requires(Robot.sparkDriveSubsystem);
		
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
		System.out.println("TICKSSSSS: " + Robot.sparkDriveSubsystem.getEncoderTicks(RobotMap.SPARK_R));
		this.absoluteTickGoal = (int)(Robot.sparkDriveSubsystem.getEncoderTicks(RobotMap.SPARK_R) + (inches * RobotMap.K_TICKS_PER_INCH));
	}

	@Override
	protected void initialize() {
		//these methods are here in initialize rather than in the constructor
		//because the robot probably updates the encoder ticks values after the constructor is called
		setDistance(moveByInches);
		prevTicks = Robot.sparkDriveSubsystem.getEncoderTicks(RobotMap.SPARK_R);
		
		System.out.println("ANGLE: " + Robot.sparkDriveSubsystem.getGyroAngle());

		System.out.println("REQUIRED TICKS: " + absoluteTickGoal);
		Robot.sparkDriveSubsystem.drive(power, power);
	}

	@Override
	protected void execute() {
		double tempPower = power;
		double currTicks = Robot.sparkDriveSubsystem.getEncoderTicks(RobotMap.SPARK_R);
		
		double delta = currTicks - prevTicks;
		
		/* If the robot has exceeded the threshold below, it will move at a slower power */
		if (Math.abs(absoluteTickGoal - (currTicks + delta)) < RobotMap.K_ENCODER_THRESHOLD) { // Math.abs() allows this to work regardless of driving direction (forwards or backwards)
			if (moveByInches > 0) tempPower = SLOW_POWER;
			else tempPower = -SLOW_POWER;
		}
		Robot.sparkDriveSubsystem.drive(tempPower,tempPower);
				
		prevTicks = currTicks;
		
		Robot.sparkDriveSubsystem.printEncoders();
		
	}

	@Override
	protected boolean isFinished() {
		double currTicks = Robot.sparkDriveSubsystem.getEncoderTicks(RobotMap.SPARK_R);
		if (this.moveByInches > 0) return (absoluteTickGoal < currTicks);
		else return (absoluteTickGoal > currTicks);
	}

	@Override
	protected void end() {
		System.out.println(power);
		System.out.println("----------------");
		System.out.println("REQUIRED TICKS: " + absoluteTickGoal);
		Robot.sparkDriveSubsystem.printEncoders();		
		System.out.println("Difference: " + Robot.sparkDriveSubsystem.encoderDifference());
		
		System.out.println();
		
		Robot.sparkDriveSubsystem.stopMotors();
	}

	@Override
	protected void interrupted() {
		Robot.sparkDriveSubsystem.stopMotors();
	}
}