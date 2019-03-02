package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.subsystems.Arm.ArmPosition;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * A class for testing a sequence of autonomous commands.
 */
public class ElevatorArmMove extends CommandGroup {

	public ElevatorArmMove(ElevatorPosition e, ArmPosition a, double elevatorPower, double armPower) {
        addParallel(new ElevatorMove(e, false, elevatorPower));
        addParallel(new ArmMove(a, armPower));
    }
	
}
