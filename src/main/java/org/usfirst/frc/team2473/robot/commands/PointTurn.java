package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;


/**
 * A command that allows for accurate robot turning to a specific angle.
 */
public class PointTurn extends Command {
	
	/**
	 * the signed power of the left motor while turning (-1 to 1)
	 */
	private double leftPower;
	
	/**
	 * the signed power of the right motor while turning (-1 to 1)
	 */
	private double rightPower;
	
	/**
	 * The amount of degrees to turn by
	 */
	private double turnByDegrees;
	
	/**
	 * Indicates whether the robot is turning clockwise based on
	 * sign of turnByDegrees.
	 */
	private boolean isClockwise;
	
	/**
	 * Variable that keeps track of the previous gyro heading
	 * (gyro heading at the end of the previous call of execute).
	 */
	private double prevAngle;
	
	/**
	 * This stores the value of the absolute final heading the robot
	 * should be facing (can be any number)
	 */
	private double angleGoal;
	
	/**
	 * This stores the value of the absolute heading the robot
	 * was facing before it started turning (can be any number)
	 */
	private double initialAngle;
	
	/**
	 * This stores the value of the initial power of the motors
	 * when turning (always positive)
	 */
	private double initialPower;	
	
	/**
	 * Creates a PointTurn object given the degrees needed to turn by
	 * and turn power.
	 * @param deltaDegrees the amount of degrees to turn by (positive is clockwise, negative is counterclockwise)
	 * @param power the starting power of the turn
	 */
	public PointTurn(double deltaDegrees, double power) {
		requires(Robot.driveSubsystem);
		
		/* power entered must only determine turn power and not direction */
		if (power < 0) throw new IllegalArgumentException("Power must be positive for point turn!");
		this.initialPower = power;
		
		/*
		* If turning 45 degrees or less, starting at greater than the
		* K_START_STALL_POWER yields and inaccurate turn. Thus, it is
		* limited over here.
		*/
		this.turnByDegrees = deltaDegrees;
		if(Math.abs(deltaDegrees) < 45) power = RobotMap.K_START_STALL_POWER;
		
		isClockwise = deltaDegrees > 0;
		setPower(power);
		
	}
	
	/**
	 * Changes the values of leftPower and rightPower based on checks for positive values
	 * and minimum power at which the motors may start stalling.
	 * @param power desired power (positive)
	 */
	private void setPower(double power) {
		/* inputed power must only be for magnitude and must be large enough to prevent motor stalls */
		if (power < 0) throw new IllegalArgumentException("Power must be positive for point turn!");
		if (power < RobotMap.K_RUNNING_STALL_POWER) power = RobotMap.K_RUNNING_STALL_POWER;
		this.leftPower = isClockwise ? power : -power;
		this.rightPower = -leftPower;
	}
	
	/**
	 * Returns the magnitude of the current power of the motors while turning
	 * @return turning power
	 */
	public double getPower(){
		return isClockwise ? leftPower : rightPower;
	}
	
	@Override
	protected void initialize() {
		prevAngle = Robot.driveSubsystem.getGyroAngle();
		this.initialAngle = prevAngle;
		this.angleGoal = prevAngle + this.turnByDegrees;
		Robot.driveSubsystem.drive(leftPower, leftPower, rightPower, rightPower);
	}

	@Override
	protected void execute() {
		double currDegrees = Robot.driveSubsystem.getGyroAngle();
		/* ensures that degreesToGoal is a positive double */
		double degreesToGoal = isClockwise ? angleGoal-currDegrees : currDegrees-angleGoal;
		
		/*
		 * Starting at 90 degrees, dampen motor power based on experimentally determined
		 * K_TURN constant and a ratio of the degrees left to turn by and the total amount
		 * of degrees needed to turn.
		 */
		if (degreesToGoal < 90) {
			double settingPower = RobotMap.K_TURN*initialPower*(degreesToGoal/(Math.abs(turnByDegrees)));
			setPower(Math.abs(settingPower));
		}
		
		
		double deltaAngle = currDegrees - prevAngle;
		boolean movingInTurnDirection = (isClockwise) ? deltaAngle > 1 : deltaAngle < -1;
		
		/*
		 * For the last 10 degrees, set the motor powers in the opposite direction to
		 * counteract any skid so the robot comes to a complete stop by the desired
		 * destination. To prevent the robot from accidentally moving in the opposite
		 * direction, keep track of the deltaAngle to confirm its direction of movement
		 * and set motor powers accordingly.
		 */
		if (degreesToGoal <= 10 && movingInTurnDirection) {
			if (isClockwise) {
				Robot.driveSubsystem.drive(-RobotMap.K_OPPOSITE_POWER, -RobotMap.K_OPPOSITE_POWER, RobotMap.K_OPPOSITE_POWER, RobotMap.K_OPPOSITE_POWER);
			} else {
				Robot.driveSubsystem.drive(RobotMap.K_OPPOSITE_POWER, RobotMap.K_OPPOSITE_POWER, -RobotMap.K_OPPOSITE_POWER, -RobotMap.K_OPPOSITE_POWER);
			}
		}else {
			Robot.driveSubsystem.drive(leftPower, leftPower, rightPower, rightPower);
		}
		
		//System.out.printf("Power: %-5.3f | DTG: %.3f \n", Devices.getInstance().getTalon(RobotMap.TALON_BL).get(), degreesToGoal);
		
		prevAngle = currDegrees;
		
	}

	@Override
	protected boolean isFinished() {
		double currAngle = Robot.driveSubsystem.getGyroAngle();
		return isClockwise ? currAngle > angleGoal : currAngle < angleGoal;
	}

	@Override
	protected void end() {
		
		double angle = Robot.driveSubsystem.getGyroAngle();
		
		System.out.println("Absolute Angle: "+ angle);
				
		Robot.driveSubsystem.stopMotors();
		System.out.println("Relative Angle: " + Math.abs(initialAngle-angle));
				
		System.out.println("Turn Speed: " + this.initialPower);
		System.out.println("Current speed: " + this.leftPower);
	}
	
	@Override
	protected void interrupted() {
		Robot.driveSubsystem.stopMotors();
	}
}