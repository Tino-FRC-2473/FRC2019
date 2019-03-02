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

	private JoystickButton elevatorPickupButton;
	private JoystickButton elevatorLowButton;
	private JoystickButton elevatorMidButton;
	private JoystickButton elevatorHighButton;
	
	private JoystickButton elevatorUpButton;
    private JoystickButton elevatorDown;
    
    private JoystickButton releaseElementButton;

    private JoystickButton armUpButton;
    private JoystickButton armDownButton;

	public OI() {
        throttle = new Joystick(2);	
        releaseElementButton = new JoystickButton(throttle, 7);
        

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

		elevatorPickupButton = new JoystickButton(buttonPanel, 1);
		elevatorLowButton = new JoystickButton(buttonPanel, 3);
		elevatorMidButton = new JoystickButton(buttonPanel, 5);
		elevatorHighButton = new JoystickButton(buttonPanel, 7);

		elevatorUpButton = new JoystickButton(buttonPanel, 6);
		elevatorDown = new JoystickButton(buttonPanel, 8);

        armUpButton = new JoystickButton(buttonPanel, 4);
        armDownButton = new JoystickButton(buttonPanel, 10); // temp change this


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
    
    public JoystickButton getReleaseElementButton() {
        return releaseElementButton;
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

	public JoystickButton getElevatorPickupButton() {
		return elevatorPickupButton;
	}

	public JoystickButton getElevatorLowButton() {
		return elevatorLowButton;
	}

	public JoystickButton getElevatorMidButton() {
		return elevatorMidButton;
	}

	public JoystickButton getElevatorHighButton() {
		return elevatorHighButton;
	}

	public JoystickButton getElevatorUpButton() {
		return elevatorUpButton;
	}

	public JoystickButton getElevatorDown() {
		return elevatorDown;
	}

	public JoystickButton getArmUpButton() {
		return armUpButton;
    }
    
    public JoystickButton getArmDownButton() {
		return armDownButton;
	}
}