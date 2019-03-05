/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.subsystems;

import org.usfirst.frc.team2473.framework.CHS_SparkMax;
import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.commands.ElevatorMove;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class contains all components of the robot necessary for driving.
 */
public class Elevator extends Subsystem {
    
    public enum ElevatorPosition {
        //Units are encoder ticks. 
        ZERO(0), CARGO_LOW(0), CARGO_MID(94), CARGO_HIGH(191), CARGO_PICKUP(0), CARGO_GROUND(0), HATCH_LOW(17.8), HATCH_MID(117.5), HATCH_HIGH(197), HATCH_PICKUP(0);
        // ZERO(0), HATCH_PICKUP(3.9), HATCH_LOW(11), HATCH_MID(111.334), HATCH_HIGH(193.87), CARGO_PICKUP(48.07), CARGO_LOW(4.071), CARGO_MID(105.550), CARGO_HIGH(199.537);
        //ZERO(0), HATCH_PICKUP(3.9), HATCH_LOW(11), HATCH_MID(111.334), HATCH_HIGH(203.87), CARGO_PICKUP(48.07), CARGO_LOW(4.071), CARGO_MID(105.550), CARGO_HIGH(199.537);

        // first: 513
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
    
    private CHS_SparkMax spark; 

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
        spark.set(0);
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
        return Math.abs(spark.getSparkMaxObject().get()) != 0;
    }
    
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initDefaultCommand() {}
	
	
}
