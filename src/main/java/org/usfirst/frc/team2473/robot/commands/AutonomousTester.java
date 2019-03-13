package org.usfirst.frc.team2473.robot.commands;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Arm.ArmPosition;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
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

    public void addDiagnosticTests() {
        addSequential(new CVLightOn());
        addSequential(new RunSparkRaw(0.5, 1));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new RunSparkRaw(-0.5, 1));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ElevatorZero());
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ArmZero());
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(3));
        addSequential(new CVLightOn());
        addSequential(new ArmMove(ArmPosition.CARGO_PICKUP, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ArmMove(ArmPosition.STOW, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ElevatorMove(ElevatorPosition.HATCH_LOW, false, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ElevatorMove(ElevatorPosition.HATCH_MID, false, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ArmMove(ArmPosition.HATCH_MID, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ElevatorMove(ElevatorPosition.HATCH_LOW, false, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ArmMove(ArmPosition.STOW, 0.5));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new RunRoller(1, 1));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new RunRoller(-1, 1));
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ElevatorZero());
        addSequential(new CVLightOff());
        addSequential(new WaitCommand(1));
        addSequential(new CVLightOn());
        addSequential(new ArmZero());
        addSequential(new CVLightOff());
        
    }

    public void addArmRampTests(double power) {
        addSequential(new ArmZero());
        addSequential(new WaitCommand(3));
        addSequential(new ArmMove(ArmPosition.CARGO_GROUND, power, 1));
        addSequential(new ArmRampDown());
        // addSequential(new WaitCommand(1));
        addSequential(new ArmMove(ArmPosition.STOW, power));
    }

    public void addArmTester(double power) {
        addSequential(new ArmZero());
        addSequential(new WaitCommand(3));
        addSequential(new ArmMove(ArmPosition.CARGO_PICKUP, power));
        addSequential(new WaitCommand(1));
        addSequential(new ArmMove(ArmPosition.STOW, power));
        addSequential(new WaitCommand(1));
        addSequential(new ArmMove(ArmPosition.CARGO_PICKUP, power));
        addSequential(new WaitCommand(1));
        addSequential(new ArmMove(ArmPosition.STOW, power));
        addSequential(new WaitCommand(1));
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

class CVLightOn extends InstantCommand {
    @Override
    protected void execute() {
        Robot.cvLight.set(Value.kForward);
    }
}

class CVLightOff extends InstantCommand {
    @Override
    protected void execute() {
        Robot.cvLight.set(Value.kOff);
    }
}