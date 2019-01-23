/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.subsystems;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class contains all components of the robot necessary for driving.
 */
public class Elevator extends Subsystem {
    
    public enum ElevatorPosition {
        ZERO(0), BASE(100), FIRST(220), SECOND(400), THIRD(600);

        private final int value;

        private ElevatorPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

	private static Elevator instance;
	
	static {
		instance = new Elevator();
	}
    
    private WPI_TalonSRX talon;

    private boolean encoderResetComplete;
	/**
	 * Gets the current instance.
	 * @return current instance of DriveSubsystem
	 */
	public static Elevator getInstance() {
		return instance;
	}
	
	
	private Elevator() {
		talon = Devices.getInstance().getTalon(RobotMap.TALON_ELEVATOR);
	}
    
    public void set(double speed) {
        System.out.println("Setting " + speed);
        talon.set(speed);
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
