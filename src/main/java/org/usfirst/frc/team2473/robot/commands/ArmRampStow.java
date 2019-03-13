package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * A class that ramps down power of arm and then starts an new ArmMove
 */
public class ArmRampStow extends CommandGroup {
	
	public ArmRampStow(ElevatorArmMove e){
        addSequential(new ArmRampDown());
        addSequential(e);
    }
}
