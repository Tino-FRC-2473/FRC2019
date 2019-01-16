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

/**
 * A class that sets the talons to specific powers upon current joystick positions
 */
public class TeleopDrive2 extends Command {
	
	private AlignToHatch alignToHatch;
    WPI_TalonSRX t = new WPI_TalonSRX(11);
	public TeleopDrive2() {
		requires(Robot.driveSubsystem);
	}

	@Override
	protected void initialize() {
		t.setSelectedSensorPosition(0,0,0);
		alignToHatch = new AlignToHatch();

		Robot.oi.getB0().whenPressed(new ElevatorMove(ElevatorPosition.BASE, 0.2));
		Robot.oi.getB1().whenPressed(new ElevatorMove(ElevatorPosition.FIRST, 0.2));
		Robot.oi.getB2().whenPressed(new ElevatorMove(ElevatorPosition.SECOND, 0.2));
		Robot.oi.getB3().whenPressed(new ElevatorMove(ElevatorPosition.THIRD, 0.2));
	}

	@Override
	protected void execute() {
        // double throttleZ = Robot.oi.getThrottle().getZ();
		// t.set(throttleZ);
		// System.out.println(t.getSelectedSensorPosition());

		
		
		Robot.elevator.printEncoders();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
