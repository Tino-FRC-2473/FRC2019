/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A class that enables accurate elevator movement to a given distance
 * at a given power.
 */
public class ElevatorMove extends Command {
	
	/**
	 * Lowest power elevator will run at towards the end of
	 * the move.
	 */
	public static final double SLOW_POWER = 0.1;
	
	/**
	 * The number of ticks that the elevator should
	 * be at if it has reached its goal.
	 */
	private int absoluteTickGoal;
	
	/**
	 * The absolute number of ticks that the elevator
	 * was at after the last call of execute.
	 */
	private int prevTicks;
	
	/**
	 * Initial power to move the elevator at.
	 */
	private double power;
	private double initialPower; //TODO may be useless

	private int initialTickDelta;

	private ElevatorPosition pos;
	
	/**
	 * Create a ElevatorMove object with given inch goal and power.
	 * @param pos ElevatorPosition to move to
	 * @param power initial power to move at
	 */
    public ElevatorMove(ElevatorPosition pos, double power) {
		requires(Robot.elevator);
		
		this.power = power;
		this.initialPower = power;
        this.pos = pos;
	}
	
	/**
	 * Set the power to move the robot at.
	 * @param power power
	 */
	public void setPower(double power) {
		this.power = power;
	}

	@Override
	protected void initialize() {
		this.initialTickDelta = this.pos.getValue()-Robot.elevator.getEncoderTicks();

		this.power = this.initialPower;

		this.power = (this.initialTickDelta < 0) ? -power : power;

		// Half the speed if going down because the elevator is going with the direction of gravity
		if (this.initialTickDelta < 0) {
			power /= 2;
		}

		this.absoluteTickGoal = Robot.elevator.getEncoderTicks() + this.initialTickDelta;
		System.out.println("Absolute tick goal: " + absoluteTickGoal);
		
		prevTicks = Robot.elevator.getEncoderTicks();
		Robot.elevator.set(power);

		System.out.println("Power: " + power);
	}

	@Override
	protected void execute() {
		double tempPower = power;
		int currTicks = Robot.elevator.getEncoderTicks();
		
		int delta = currTicks - prevTicks;
		
		/* If the elevator has exceeded the threshold below, it will move at a slower power */
		if (Math.abs(absoluteTickGoal - (currTicks + delta)) < RobotMap.K_ENCODER_ELEVATOR_THRESHOLD) { // Math.abs() allows this to work regardless of moving direction (forwards or backwards)
			if (initialTickDelta > 0) {
				tempPower = SLOW_POWER;
			} else {
				tempPower = -SLOW_POWER;
			}
		}
		Robot.elevator.set(tempPower);
				
		prevTicks = currTicks;
		
		
	}

	@Override
	protected boolean isFinished() {
		int currTicks = Robot.elevator.getEncoderTicks();
		if (this.initialTickDelta > 0) return (absoluteTickGoal < currTicks);
		else return (absoluteTickGoal > currTicks);
	}

	@Override
	protected void end() {
		System.out.println(power);
		System.out.println("----------------");
		System.out.println("REQUIRED TICKS: " + absoluteTickGoal);
		Robot.elevator.printEncoders();		
		
		System.out.println();
		
		Robot.elevator.stop();
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}