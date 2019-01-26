package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * A class for testing a sequence of autonomous commands.
 */
public class AutonomousTester extends CommandGroup {
	
	/**
	 * the amount of time to wait in between commands (seconds)
	 */
	private final double WAIT = 0.1;

	/**
	 * Constructs an empty tester.
	 */
	public AutonomousTester() {
    }
	
	/**
	 * Adds a sequence to drive forward, turn, and drive forward.
	 * @param distanceFirst 	the distance to drive before turning (inches)
	 * @param turnAngle 		the angle to turn (degrees)
	 * @param distanceSecond	the distance to drive after turning (inches)
	 */
	public void addDriveTurnDrive(double distanceFirst, int turnAngle, double distanceSecond) {
		addSequential(new StraightDrive(distanceFirst, 0.3));
		addSequential(new WaitCommand(WAIT));
		addSequential(new PointTurn(turnAngle, 0.45));
		addSequential(new WaitCommand(WAIT));
		addSequential(new StraightDrive(distanceSecond, 0.3));	
	}
	
	public void addCV() {
		addSequential(new StraightDrive(48, 0.3));
		addSequential(new WaitCommand(WAIT));
		addSequential(new PointTurn(90, 0.45));
		addSequential(new WaitCommand(WAIT));
		addSequential(new AlignToHatch());	
	}

	/**
	 * Adds a sequence to drive in a lightning bolt shape.
	 * 
	 * The lightning bolt consists of the following actions:
	 * 		- drive straight 60 inches
	 * 		- turn right 45 degrees
	 * 		- drive straight 30 inches
	 * 		- turn left 45 degrees
	 * 		- drive straight 30 inches
	 */
	public void addLightningBolt() {
		addSequential(new WaitCommand(WAIT));
    	
		addSequential(new StraightDrive(60, 0.3));
		addSequential(new WaitCommand(WAIT));
        
		addSequential(new PointTurn(45, 0.45));
		addSequential(new WaitCommand(WAIT));
        
		addSequential(new StraightDrive(30, 0.3));
		addSequential(new WaitCommand(WAIT));
        
		addSequential(new PointTurn(-45, 0.45));
		addSequential(new WaitCommand(WAIT));
        
		addSequential(new StraightDrive(30, 0.3));
	}

	/**
	 * Adds a sequence to drive in a clockwise 48x48 inch square. 
	 */
	public void addSquare() {
		for (int i = 0; i < 4; i++) {
    		addSequential(new StraightDrive(48, 0.3));
    		addSequential(new WaitCommand(WAIT));
    		addSequential(new PointTurn(90, 0.45));
    		addSequential(new WaitCommand(WAIT));
    	}
	}
}
