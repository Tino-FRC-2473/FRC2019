/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.State;
import org.usfirst.frc.team2473.robot.Robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Cargo extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public static enum BallEvent {
		NONE, UNSAFE, SAFE, CAPTURED
	}

	public static enum RequestEvent {
		RELEASE
	}

	public static double UNSAFE_VOLTAGE_MIN = 1.2;
	public static double UNSAFE_VOLTAGE_MAX = 1.5;
	public static double CAPTURE_VOLTAGE = 1.8;

	private static final double POWER_UP_FAST = 0.8;
	private static final double POWER_DOWN_SLOW = -0.5;

	public final State RELEASING = new State("Releasing") {

		@Override
		public void init() {
			cargoTalon.set(POWER_UP_FAST);
		}

		@Override
		public State handleEvent(Enum event) {
			if (event instanceof BallEvent) {
				switch ((BallEvent) event) {
					case NONE:
						return REARMING;
					case CAPTURED:
					case SAFE:
					case UNSAFE:
						break;
					default:
						break;
				}
			}
			return null;
		}

	};

	public final State REARMING = new State("Rearming") {

		@Override
		public void init() {

		}

		@Override
		public State handleEvent(Enum event) {
			if (event instanceof BallEvent) {
				switch ((BallEvent) event) {
					case SAFE:
					case CAPTURED:
						return CAPTURING;
					default:
						break;
				}
			}
			return null;
		}

	};

	public final State CAPTURING = new State("Capturing") {

		@Override
		public void init() {
			cargoTalon.set(POWER_DOWN_SLOW);
		}

		@Override
		public State handleEvent(Enum event) {
			if (event instanceof BallEvent) {
				switch ((BallEvent) event) {
					case CAPTURED:
					case SAFE:
						cargoTalon.set(POWER_DOWN_SLOW);
						break;
					case UNSAFE:
						//return RELEASING;
						cargoTalon.set(0);
						break;
					case NONE:
						return RELEASING;
					default:
						break;
				}
			} else if (event instanceof RequestEvent) {
				if ((RequestEvent) event == RequestEvent.RELEASE) {
					return RELEASING;
				}
			}
			return null;
		}

	};

	WPI_TalonSRX cargoTalon;
	AnalogInput sharpDistanceMotorSide;
	AnalogInput sharpDistanceLimitSide;

	private static Cargo theInstance;

	private State state;

	static {
		theInstance = new Cargo();
	}

	public static Cargo getInstance() {
		return theInstance;
	}

	private Cargo() {
		cargoTalon = Devices.getInstance().getTalon(10);
		sharpDistanceMotorSide = new AnalogInput(1);
		sharpDistanceLimitSide = new AnalogInput(2);
	}

	// public boolean getFwdLimitSwitch() {
	// 	return !cargoTalon.getSensorCollection().isFwdLimitSwitchClosed();
	// }
	
	// public boolean getRevLimitSwitch() {
	// 	return !cargoTalon.getSensorCollection().isRevLimitSwitchClosed();
	// }

	public double getSharpVoltageMotorSide() {
		// double inches = (26 / sharpDistance.getAverageVoltage()) / 2.54; // conversion factor from spec sheet
		// double coeff = 5.0/6.0;
		// return inches * coeff;

		return sharpDistanceMotorSide.getVoltage();
	}

	public double getSharpVoltageLimitSide() {
		// Scale to values of sharpDistanceMotorSide. This conversion will be different for each distance sensor

		// newVoltage = (1.07 * currentVoltage) - 0.0184

		return (1.07 * sharpDistanceLimitSide.getVoltage()) - 0.0184;
	}

	public void setPower(double speed) {
		cargoTalon.set(speed);
	}

	public void stop() {
		cargoTalon.set(0);
	}

	public void setState(State state) {
		this.state = state;
		System.out.println("SWITCHING TO " + this.state);
		this.state.init();
	}

	public State getState() {
		return state;
	}

	@Override
	protected void initDefaultCommand() {

	}
}
