package org.usfirst.frc.team2473.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	private Joystick throttle;
	private Joystick wheel;

	private JoystickButton cvButton;
	private JoystickButton reverseDriveButton;

	public OI() {
		throttle = new Joystick(2);		
		wheel = new Joystick(0);

		cvButton = new JoystickButton(wheel, 1);
		reverseDriveButton = new JoystickButton(wheel, 2);		
	}
	
	public Joystick getThrottle() {
		return throttle;
	}

	public Joystick getWheel() {
		return wheel;
	}

	public JoystickButton getCVButton() {
		return cvButton;
	}

	public JoystickButton getReverseDriveButton() {
		return reverseDriveButton;
	}

}