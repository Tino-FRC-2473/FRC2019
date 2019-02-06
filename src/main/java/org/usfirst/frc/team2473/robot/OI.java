package org.usfirst.frc.team2473.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	private Joystick throttle;
	private Joystick wheel;
	private JoystickButton b0;
	private JoystickButton b1;
	private JoystickButton b2;
	private JoystickButton b3;
	private JoystickButton b4;

    private JoystickButton cvButton;
    private JoystickButton reverseDriveButton;

	public OI() {
		throttle = new Joystick(2);		
		wheel = new Joystick(0);

		b0 = new JoystickButton(wheel, 1);
		b1 = new JoystickButton(wheel, 2);
		b2 = new JoystickButton(wheel, 3);
		b3 = new JoystickButton(wheel, 4);
		b4 = new JoystickButton(wheel, 5);
        
		cvButton = new JoystickButton(wheel, 6);
		reverseDriveButton = new JoystickButton(wheel, 7);		
	}
	
	public Joystick getThrottle() {
		return throttle;
	}

	public Joystick getWheel() {
		return wheel;
	}

	public JoystickButton getB0() {
		return b0;
	}

	public JoystickButton getB1() {
		return b1;
	}

	public JoystickButton getB2() {
		return b2;
	}

	public JoystickButton getB3() {
		return b3;
	}

	public JoystickButton getB4() {
		return b4;

  public JoystickButton getCVButton() {
		return cvButton;
  }
    
	public JoystickButton getReverseDriveButton() {
		return reverseDriveButton;
	}
}