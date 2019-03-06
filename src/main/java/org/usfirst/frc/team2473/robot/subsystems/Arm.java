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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class contains all components of the robot necessary for driving.
 */
public class Arm extends Subsystem {
    
    
    public static AnalogInput distanceSensor;
    public static boolean allowZero = false;
    public double lastNonZeroPower = 0;

    public enum ArmPosition {
        //Units are encoder ticks. ground: -48.8
        ZERO(0), STOW(-5), CARGO_LOW(-19.6904), CARGO_MID(-19.6904), CARGO_HIGH(-19.6904), CARGO_PICKUP(-5), CARGO_GROUND(-47.3), HATCH_LOW(-44.69), HATCH_MID(-44.69), HATCH_HIGH(-39.4), HATCH_PICKUP(-5);

        private final double value;

        /**
         * @param value refers to the number of encoder ticks of a certain position
         */
        private ArmPosition(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    private ArmPosition executingGoalPosition;

	private static Arm instance;
	
	static {
        instance = new Arm();
		distanceSensor = new AnalogInput(0);
	}
    
    private CHS_SparkMax spark; 

    private boolean encoderResetComplete;

	/**
	 * Gets the current instance.
	 * @return current instance of Elevator
	 */
	public static Arm getInstance() {
		return instance;
	}
	
	private Arm() {
		spark = new CHS_SparkMax(RobotMap.TALON_ARM, MotorType.kBrushless);
        setExecutingGoalPosition(ArmPosition.ZERO);
    }

    public ArmPosition getExecutingGoalPosition() {
        return executingGoalPosition;
    }
    
    public void setExecutingGoalPosition(ArmPosition newPosition) {
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

    public double getPower() {
        return spark.getSparkMaxObject().get();
    }
    
    public void set(double speed) {
        //System.out.println("Setting " + speed);
        if (!allowZero && speed > 0 && getEncoderTicks() > -5) {
            stop();
        } else if (!allowZero && speed < 0 && getEncoderTicks() < -49) {
            stop();
        } else {
            if (speed != 0) {
                lastNonZeroPower = speed;
                System.out.println("NON_ZERO: " + lastNonZeroPower);
            }
            spark.set(speed);
        }
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
		System.out.println("Arm: " + getEncoderTicks());
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