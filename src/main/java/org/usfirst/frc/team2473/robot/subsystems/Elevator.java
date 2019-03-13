package org.usfirst.frc.team2473.robot.subsystems;

import org.usfirst.frc.team2473.framework.CHS_SparkMax;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.commands.ElevatorMove;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class contains all components of the robot necessary for moving the Elevator.
 */
public class Elevator extends Subsystem {
    
    public enum ElevatorPosition {
        ZERO(0), CARGO_LOW(0), CARGO_MID(99), CARGO_HIGH(191), CARGO_PICKUP(0), CARGO_GROUND(0), HATCH_LOW(17.8), HATCH_MID(117.5), HATCH_HIGH(197), HATCH_PICKUP(0);
        
        private final double value;

        /**
         * @param value refers to the number of encoder ticks of a certain position
         */
        private ElevatorPosition(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    private ElevatorPosition executingGoalPosition;

	private static Elevator instance;
	
	static {
		instance = new Elevator();
	}
    
    public CHS_SparkMax spark; 

    private boolean encoderResetComplete;

	/**
	 * Gets the current instance.
	 * @return current instance of Elevator
	 */
	public static Elevator getInstance() {
		return instance;
	}
	
	private Elevator() {
        spark = new CHS_SparkMax(RobotMap.TALON_ELEVATOR, MotorType.kBrushless);
        setExecutingGoalPosition(ElevatorPosition.ZERO);
    }

    public ElevatorPosition getExecutingGoalPosition() {
        return executingGoalPosition;
    }
    
    public void setExecutingGoalPosition(ElevatorPosition newPosition) {
        this.executingGoalPosition = newPosition;
    }

    public boolean isLowerLimitSwitchPressed() {

        // is reverse closed
        return spark.getSparkMaxObject().getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean isUpperLimitSwitchPressed() {

        // is reverse closed
        return spark.getSparkMaxObject().getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed).get();
    }
    
    public void set(double speed) {
        //System.out.println("Setting " + speed);
        spark.set(speed);

        if(Math.abs(speed) >= ElevatorMove.SLOW_POWER - 0.01)
            encoderResetComplete = false;
    }

    public void stop() {
        spark.set(0.01);
    }

    public double getEncoderTicks() {
        return spark.getEncoderPosition();
    }

    public synchronized void resetEncoders() {
        spark.getSparkMaxObject().getEncoder().setPosition(0);
        encoderResetComplete = true;
    }

    public synchronized boolean isEncoderResetComplete() {
		return encoderResetComplete;
	}
    
    public void printEncoders() {
		System.out.println("Elevator: " + getEncoderTicks());
    }

    public boolean isMoving() {
        // System.out.println(spark.getSparkMaxObject().get());
        return Math.abs(spark.getSparkMaxObject().get()) > 0.01;
    }
    
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initDefaultCommand() {}
	
	
}
