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
        // ZERO(0), RELEASE_CARGO_MECH(7), CARGO_LOW(2.4), CARGO_MID(98.6), CARGO_HIGH(191), CARGO_PICKUP(0), CARGO_GROUND(0), HATCH_LOW(22.4), HATCH_MID(118.6), HATCH_HIGH(197), HATCH_PICKUP(0);
        ZERO(0), RELEASE_CARGO_MECH(7), CARGO_LOW(9.4), CARGO_MID(105.6), CARGO_HIGH(198), CARGO_PICKUP(0), CARGO_GROUND(0), HATCH_LOW(24.4), HATCH_MID(125.6), HATCH_HIGH(204), HATCH_PICKUP(7);

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
    
    private boolean wasLimitSwitchPressedLast = false;

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

        if ((speed < 0 && getEncoderTicks() < 10) || (speed > 0 && getEncoderTicks() > 190)) {
            int sign = (speed < 0) ? -1 : 1;
            spark.set(sign * 0.3);
        }

        if(Math.abs(speed) >= ElevatorMove.SLOW_POWER - 0.01)
            encoderResetComplete = false;

        if (isLowerLimitSwitchPressed() && !wasLimitSwitchPressedLast) {
            resetEncoders();
            wasLimitSwitchPressedLast = true;
        } else {
            wasLimitSwitchPressedLast = false;
        }
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
