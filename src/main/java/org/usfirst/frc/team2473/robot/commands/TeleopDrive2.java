/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;


import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

/**
 * A class that sets the talons to specific powers upon current joystick positions
 */
public class TeleopDrive2 extends Command {
	

	// Using wheel for elevator testing, not for actual control
	boolean useWheel = false;

	ElevatorMove move;

	public TeleopDrive2(boolean useWheel) {
		requires(Robot.elevator);

		this.useWheel = useWheel;
	}

	@Override
	protected void initialize() {
		if (!useWheel) {
			double power = 0.4;

			Robot.oi.getB0().whenPressed(new ElevatorMove(ElevatorPosition.BASE, power));
			Robot.oi.getB1().whenPressed(new ElevatorMove(ElevatorPosition.FIRST, power));
			Robot.oi.getB2().whenPressed(new ElevatorMove(ElevatorPosition.SECOND, power));
			Robot.oi.getB3().whenPressed(new ElevatorMove(ElevatorPosition.THIRD, power));
			Robot.oi.getB4().whenPressed(new ElevatorMove(ElevatorPosition.ZERO, power));
		}	
	}

	@Override
	protected void execute() {

        double wheelX = Robot.oi.getWheel().getX();
		
		if (useWheel) {
			Robot.elevator.set(wheelX);
		}
		
		Robot.elevator.printEncoders();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
