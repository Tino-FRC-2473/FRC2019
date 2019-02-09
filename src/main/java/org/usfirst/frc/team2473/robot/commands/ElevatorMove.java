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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	private int shuffleboardTestTicks = 0;

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
		SmartDashboard.putBoolean("Elevator Status", true);
	}

	@Override
	protected void execute() {
		double tempPower = power;
		//int currTicks = Robot.elevator.getEncoderTicks();
		
		int currTicks = shuffleboardTestTicks;
		shuffleboardTestTicks+=(int)(Math.random()*10);
		System.out.println(currTicks);
		
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
		//Checks if while executing, elevator move is moving by a minimum number of ticks per cycle.
		if(delta < RobotMap.ELEVATOR_MIN_TICKS){
			SmartDashboard.putBoolean("Elevator Status", false);
		}else if (!SmartDashboard.getBoolean("Elevator Status", true) && delta > RobotMap.ELEVATOR_MIN_TICKS){
			SmartDashboard.putBoolean("Elevator Status", true);
		}

		//sets the elevator value in shuffleboard based on ticks
		//currTicks = Robot.elevator.getEncoderTicks();
		//currTicks = test;
		double diff = 0;
		double gap = 0;
		double value = 0;
		if(currTicks==ElevatorPosition.ZERO.getValue()){
			SmartDashboard.putNumber("Elevator", -1);
		}
		else if(currTicks<ElevatorPosition.BASE.getValue()){
			diff = currTicks-ElevatorPosition.ZERO.getValue();
			gap = ElevatorPosition.BASE.getValue()-ElevatorPosition.ZERO.getValue();
			value = diff/gap;
			SmartDashboard.putNumber("Elevator",-1+value);
		}
		else if(currTicks==ElevatorPosition.BASE.getValue()){
			SmartDashboard.putNumber("Elevator",0);
		}
		else if(currTicks<ElevatorPosition.FIRST.getValue()){
			diff = currTicks-ElevatorPosition.BASE.getValue();
			gap = ElevatorPosition.FIRST.getValue()-ElevatorPosition.BASE.getValue();
			value = diff/gap;
			SmartDashboard.putNumber("Elevator",value);
		}
		else if(currTicks==ElevatorPosition.FIRST.getValue()){
			SmartDashboard.putNumber("Elevator",1);
		}
		else if(currTicks<ElevatorPosition.SECOND.getValue()){
			diff = currTicks-ElevatorPosition.FIRST.getValue();
			gap = ElevatorPosition.SECOND.getValue()-ElevatorPosition.FIRST.getValue();
			value = diff/gap;
			SmartDashboard.putNumber("Elevator",1+value);
		}
		else if(currTicks==ElevatorPosition.SECOND.getValue()){
			SmartDashboard.putNumber("Elevator",2);
		}
		else if(currTicks<ElevatorPosition.THIRD.getValue()){
			diff = currTicks-ElevatorPosition.SECOND.getValue();
			gap = ElevatorPosition.THIRD.getValue()-ElevatorPosition.SECOND.getValue();
			value = diff/gap;
			SmartDashboard.putNumber("Elevator",2+value);
		}
		else if(currTicks==ElevatorPosition.THIRD.getValue()){
			SmartDashboard.putNumber("Elevator",3);
		}


		
	}

	@Override
	protected boolean isFinished() {
		int currTicks = Robot.elevator.getEncoderTicks();
		if (this.initialTickDelta > 0) 
			return (absoluteTickGoal < currTicks);
		else 
			return (absoluteTickGoal > currTicks);
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