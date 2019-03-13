package org.usfirst.frc.team2473.robot.commands;
import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A class that allows for testing of lowest possible motor
 * power at which the robot won't stall after it has been
 * moving for a while.
 */
public class StallExperiment extends Command {
    
    /**
     * The initial power to start the experiment at.
     */
    private double power;
    
    /**
     * The decrement in motors power after every 360 degrees.
     */
    private double powerDecrement;
    
    /**
     * Absolute gyro angle at the end of the previous 360 degree turn.
     */
    private double prevAngle;
    
    /**
     * Absolute gyro angle at the beginning of the latest execute method call.
     */
    private double currAngle;
    
    

    /**
     * Creates a Stall Experiment at a given start power with a decrement
     * of powerDecrement after each 360 degree turn
     * @param power starting power
     * @param powerDecrement power decrement after each 360 degree turn
     */
    public StallExperiment(double power, double powerDecrement) {
        requires(Robot.driveSubsystem);
        
        if (power < 0) throw new IllegalArgumentException("Power must be positive!");
        if (powerDecrement < 0) throw new IllegalArgumentException("Change must be positive!");
        this.power = power;
        this.powerDecrement = powerDecrement;
        
    }
    
    @Override
    protected void initialize() {
        prevAngle = Devices.getInstance().getNavXGyro().getAngle();
        Robot.driveSubsystem.drive(power, -power);
    }
    
    @Override
    protected void execute() {
        currAngle = Devices.getInstance().getNavXGyro().getAngle();
        
        if (currAngle - prevAngle > 360) {
            power -= powerDecrement;
            prevAngle = currAngle;
            System.out.println("New power: " + power);
        }
        
        Robot.driveSubsystem.drive(power, -power);
        
    }

    @Override
    protected boolean isFinished() {
        return power<=0;
    }

    @Override
    protected void end() {
        System.out.println("FINAL POWER: " + power);
        Robot.driveSubsystem.stopMotors();
        
    }

    @Override
    protected void interrupted() {
        Robot.driveSubsystem.stopMotors();
    }
} 