/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.subsystems;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.commands.ElevatorMove;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class contains all components of the robot necessary for driving.
 */
public class Elevator extends Subsystem {
    
    public enum ElevatorPosition {
        //Units are encoder ticks. 
        //TODO These are now just dummy values
        ZERO(0), HATCH_PICKUP(237), HATCH_LOW(460), HATCH_MID(1600), HATCH_HIGH(2800), CARGO_PICKUP(123), CARGO_LOW(456), CARGO_MID(789), CARGO_HIGH(1023), INITIAL_STOW(2200);

        // first: 513
        private final int value;

        /**
         * @param value refers to the number of encoder ticks of a certain position
         */
        private ElevatorPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private ElevatorPosition executingGoalPosition;

	private static Elevator instance;
	
	static {
		instance = new Elevator();
	}
    
    private WPI_TalonSRX talon; 

    private boolean encoderResetComplete;

	/**
	 * Gets the current instance.
	 * @return current instance of Elevator
	 */
	public static Elevator getInstance() {
		return instance;
	}
	
	private Elevator() {
		talon = Devices.getInstance().getTalon(RobotMap.TALON_ELEVATOR);
        setExecutingGoalPosition(ElevatorPosition.ZERO);
    }

    public ElevatorPosition getExecutingGoalPosition() {
        return executingGoalPosition;
    }
    
    public void setExecutingGoalPosition(ElevatorPosition newPosition) {
        this.executingGoalPosition = newPosition;
    }

    public boolean isLowerLimitSwitchPressed() {
        return !talon.getSensorCollection().isRevLimitSwitchClosed();
    }
    
    public void set(double speed) {
        //System.out.println("Setting " + speed);
        talon.set(speed);

        if(Math.abs(speed) >= ElevatorMove.SLOW_POWER - 0.01)
            encoderResetComplete = false;
    }

    public void stop() {
        talon.set(0);
    }

    public int getEncoderTicks() {
        return talon.getSelectedSensorPosition(0);
    }

    public synchronized void resetEncoders() {
        talon.setSelectedSensorPosition(0, 0, 0);
        encoderResetComplete = true;
    }

    public synchronized boolean isEncoderResetComplete() {
		return encoderResetComplete;
	}
    
    public void printEncoders() {
		System.out.println("Elevator: " + getEncoderTicks());
    }
    
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initDefaultCommand() {}
	
	
}
