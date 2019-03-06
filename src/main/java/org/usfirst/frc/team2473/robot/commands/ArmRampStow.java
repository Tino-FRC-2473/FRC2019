package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.subsystems.Arm.ArmPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * A class for testing a sequence of autonomous commands.
 */
public class ArmRampStow extends CommandGroup {
	
	/**
	 * the amount of time to wait in between commands (seconds)
	 */

	/**
	 * Constructs an empty tester.
	 */
	public ArmRampStow(ElevatorArmMove e){
        addSequential(new ArmRampDown());
        // addSequential(new WaitCommand(0.25));
        addSequential(e);
    }
}
