package org.usfirst.frc.team2473.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
	private Joystick throttle;
	private Joystick wheel;

	private JoystickButton elevatorUp;
	private JoystickButton elevatorDown;
	private JoystickButton armUp;
	private JoystickButton armDown;

    private JoystickButton cvButton;
    private JoystickButton armResetButton;
	private Joystick buttonPanel;
	
    private JoystickButton shiftElevatorButton;

	private JoystickButton elevatorPickupButton;
	private JoystickButton elevatorLowButton;
	private JoystickButton elevatorMidButton;
	private JoystickButton elevatorHighButton;
	
	private JoystickButton groundIntakeButton;
    private JoystickButton releaseCargoButton;
	private JoystickButton manualOverrideButton;
    private JoystickButton cargoHatchToggleButton;

	public OI() {
        throttle = new Joystick(2);	
        shiftElevatorButton = new JoystickButton(throttle, 7);
        

		wheel = new Joystick(0);
		
		elevatorUp = new JoystickButton(wheel, 1);
		elevatorDown = new JoystickButton(wheel, 2);
		armUp = new JoystickButton(wheel, 4);
		armDown = new JoystickButton(wheel, 3);
        
		cvButton = new JoystickButton(wheel, 6);
		armResetButton = new JoystickButton(wheel, 5);
		
		buttonPanel = new Joystick(3);


		elevatorPickupButton = new JoystickButton(buttonPanel, 1);
		elevatorLowButton = new JoystickButton(buttonPanel, 3);
		elevatorMidButton = new JoystickButton(buttonPanel, 5);
		elevatorHighButton = new JoystickButton(buttonPanel, 7);

		groundIntakeButton = new JoystickButton(buttonPanel, 2);
		releaseCargoButton = new JoystickButton(buttonPanel, 4);
		manualOverrideButton = new JoystickButton(buttonPanel, 6);
		cargoHatchToggleButton = new JoystickButton(buttonPanel, 8);
		
		cargoHatchToggleButton.whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				RobotMap.SCORING_HATCH = !RobotMap.SCORING_HATCH;
			}
		});


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
        return shiftElevatorButton;
    }

	public Joystick getWheel() {
		return wheel;
	}

	public JoystickButton getCVButton() {
		return cvButton;
	}
	
	public JoystickButton getArmResetButton() {
		return armResetButton;
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
	
	public JoystickButton getGroundIntakeButton() {
		return groundIntakeButton;
	}

	public JoystickButton getManualOverrideButton() {
		return manualOverrideButton;
	}

	public JoystickButton getReleaseCargoButton() {
		return releaseCargoButton;
	}

	public JoystickButton getCargoHatchToggleButton() {
		return cargoHatchToggleButton;
	}
	
	public JoystickButton getElevatorUpButton() {
		return elevatorUp;
	}

	public JoystickButton getElevatorDownButton() {
		return elevatorDown;
	}

	public JoystickButton getArmUpButton() {
		return armUp;
	}

	public JoystickButton getArmDownButton() {
		return armDown;
	}
}