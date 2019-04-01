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
	public static final double SLOW_POWER = 0.6;
	
	/**
	 * The number of ticks that the elevator should
	 * be at if it has reached its goal.
	 */
	private double absoluteTickGoal;
	
	/**
	 * The absolute number of ticks that the elevator
	 * was at after the last call of execute.
	 */
	private double prevTicks;
	
	/**
	 * Initial power to move the elevator at.
	 */
    private double power;
    private boolean releaseElement;
	private double initialPower; //TODO may be useless

    private double initialTickDelta;
    private double initialTicks;

	private ElevatorPosition targetPos;
	
	/**
	 * Create a ElevatorMove object with given inch goal and power.
	 * @param pos ElevatorPosition to move to
	 * @param power initial power to move at
	 */
    public ElevatorMove(ElevatorPosition pos, boolean releaseElement, double power) {
		requires(Robot.elevator);
		
        this.power = power;
        this.releaseElement = releaseElement;
		this.initialPower = power;
		this.targetPos = pos;
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

        this.initialTicks = Robot.arm.getEncoderTicks();

        Robot.elevator.setExecutingGoalPosition(targetPos);

		if (releaseElement) {
            targetPos = Robot.elevator.getExecutingGoalPosition();

            if (Robot.elevator.getExecutingGoalPosition() == ElevatorPosition.HATCH_PICKUP) {
                this.initialTickDelta = (this.targetPos.getValue() + 25) - Robot.elevator.getEncoderTicks();
            } else if (Robot.elevator.getExecutingGoalPosition() != ElevatorPosition.ZERO) {
                this.initialTickDelta = (this.targetPos.getValue() - 10) - Robot.elevator.getEncoderTicks();
            }
        } else {
            this.initialTickDelta = this.targetPos.getValue() - Robot.elevator.getEncoderTicks();
        }

		this.power = this.initialPower;

		this.power = (this.initialTickDelta < 0) ? -power : power;

		// Half the speed if going down because the elevator is going with the direction of gravity
		if (this.initialTickDelta < 0) {
			power *= 0.75;
		}

		this.absoluteTickGoal = Robot.elevator.getEncoderTicks() + this.initialTickDelta;
		System.out.println("Absolute tick goal: " + absoluteTickGoal);
		
		prevTicks = Robot.elevator.getEncoderTicks();
		Robot.elevator.set(power);

		System.out.println("Power: " + power);
		// SmartDashboard.putBoolean("Elevator Status", true);
	}

	@Override
	protected void execute() {
		double tempPower = power;
		double currTicks = Robot.elevator.getEncoderTicks();
		
		double delta = currTicks - prevTicks;
        
        if (currTicks > ElevatorPosition.HATCH_MID.getValue()) {
            if (initialTickDelta > 0) {
                tempPower = SLOW_POWER;
            } else {
                tempPower = -SLOW_POWER;
            }
            
        }
        double P = Math.abs(initialTicks - currTicks)/RobotMap.K_ELEVATOR_RAMP_UP;
        double dampenPower = SLOW_POWER + (P *(Math.abs(tempPower) - SLOW_POWER));

		if (Math.abs(currTicks - initialTicks) < RobotMap.K_ELEVATOR_RAMP_UP) {
            if (initialTickDelta > 0) {
				tempPower = dampenPower;
			} else {
				tempPower = -dampenPower;
			}
        }

		/* If the elevator has exceeded the threshold below, it will move at a slower power */
		if (Math.abs(absoluteTickGoal - (currTicks + delta)) < RobotMap.K_ENCODER_ELEVATOR_THRESHOLD) { // Math.abs() allows this to work regardless of moving direction (forwards or backwards)
			if (initialTickDelta > 0) {
				tempPower = SLOW_POWER;
			} else {
				tempPower = -SLOW_POWER;
			}
        }
        
        


        Robot.elevator.set(tempPower);
        System.out.println("Ele power: " + tempPower);
				
		prevTicks = currTicks;
	}

	@Override
	protected boolean isFinished() {
        if (this.initialTickDelta > 0 && Robot.elevator.isUpperLimitSwitchPressed()) return true;

        double threshold = 2;
		double currTicks = Robot.elevator.getEncoderTicks();
		if (this.initialTickDelta > 0) 
			return (absoluteTickGoal - threshold < currTicks);
		else 
			return (absoluteTickGoal + threshold > currTicks);
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