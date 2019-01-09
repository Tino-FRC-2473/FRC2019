package org.usfirst.frc.team2473.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	private Joystick throttle;
	private Joystick wheel;

	public OI() {
		throttle = new Joystick(2);		
		wheel = new Joystick(0);		
	}
	
	public Joystick getThrottle() {
		return throttle;
	}

	public Joystick getWheel() {
		return wheel;
	}
}