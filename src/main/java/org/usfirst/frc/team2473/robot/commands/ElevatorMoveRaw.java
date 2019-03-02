/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class ElevatorMoveRaw extends Command {

	private double power;

	public ElevatorMoveRaw(double power) {
		requires(Robot.elevator);

		this.power = power;
	}

	@Override
	protected void initialize() {
		Robot.elevator.set(power);
	}

	@Override
	protected void execute() {
        double tempPower = power;
        if (Robot.elevator.getEncoderTicks() > ElevatorPosition.HATCH_MID.getValue()) {
            if (power > 0) {
                tempPower = 0.3;
            } else {
                tempPower = -0.3;
            }
            
        }
        Robot.elevator.set(tempPower);
	}

	@Override
	protected boolean isFinished() {
		if (power < 0) {
			return !Robot.oi.getElevatorDownButton().get();
		} else {
            //return true;
			return !Robot.oi.getElevatorUpButton().get();
		}
	}

	@Override
	protected void end() {
		Robot.elevator.stop();
	}

	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
