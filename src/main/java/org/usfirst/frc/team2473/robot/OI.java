package org.usfirst.frc.team2473.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
	private Joystick throttle;
	private Joystick wheel;

    private JoystickButton cvButton;
    private JoystickButton reverseDriveButton;
	private Joystick buttonPanel;

	private JoystickButton cargoButton;

	private JoystickButton elevatorPos1;
	private JoystickButton elevatorPos2;
	private JoystickButton elevatorPos3;
	private JoystickButton elevatorPos4;
	
	private JoystickButton elevatorInitialStow;
	private JoystickButton elevatorDown;

	private JoystickButton elevatorZero;

	public OI() {
		throttle = new Joystick(2);		
        wheel = new Joystick(0);
        
		cvButton = new JoystickButton(wheel, 6);
        reverseDriveButton = new JoystickButton(wheel, 2);	
        
        reverseDriveButton.whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				RobotMap.RUNNING_FORWARD = !RobotMap.RUNNING_FORWARD;
			}
		});
		
		buttonPanel = new Joystick(3);

		cargoButton = new JoystickButton(buttonPanel, 2);

		elevatorPos1 = new JoystickButton(buttonPanel, 7);
		elevatorPos2 = new JoystickButton(buttonPanel, 5);
		elevatorPos3 = new JoystickButton(buttonPanel, 3);
		elevatorPos4 = new JoystickButton(buttonPanel, 1);

		elevatorInitialStow = new JoystickButton(buttonPanel, 6);
		elevatorDown = new JoystickButton(buttonPanel, 8);

		elevatorZero = new JoystickButton(buttonPanel, 4);


		/*

		Button panel:

		-----------------------------

		elev pos 4			cargo
		
		elev pos 3			elev zero

		elev pos 2			elev up

		elev pos 1			elev down

		------------------------------

		*/
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

	public JoystickButton getCargoButton() {
		return cargoButton;
	}

	public JoystickButton getElevatorPos1() {
		return elevatorPos1;
	}

	public JoystickButton getElevatorPos2() {
		return elevatorPos2;
	}

	public JoystickButton getElevatorPos3() {
		return elevatorPos3;
	}

	public JoystickButton getElevatorPos4() {
		return elevatorPos4;
	}

	public JoystickButton getElevatorInitialStow() {
		return elevatorInitialStow;
	}

	public JoystickButton getElevatorDown() {
		return elevatorDown;
	}

	public JoystickButton getElevatorZero() {
		return elevatorZero;
	}
}