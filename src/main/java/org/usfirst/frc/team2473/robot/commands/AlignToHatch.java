/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.robot.subsystems.Elevator.ElevatorPosition;
import org.usfirst.frc.team2473.framework.JetsonPort;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {

    double normalPower = 0.3;
    double turnPower = 0.08;
    double addedPower = 0.1;
    private double angle = 0;
    private double distance = 0;

    private boolean hasMovedUp;

    private ElevatorMove move = null;

    public AlignToHatch() {
        requires(Robot.driveSubsystem);
    }

    @Override
    protected void execute() {
        move();
    }

    public void move() {
        if (!RobotMap.CV_RUNNING) return;
        

		double thresholdAngle = 3;
        angle = Robot.jetsonPort.getVisionAngle();
        distance = Robot.jetsonPort.getVisionDistance();
        if (!RobotMap.RUNNING_FORWARD) angle = -angle;
        if (distance == 0) {
            if (!hasMovedUp && RobotMap.RUNNING_FORWARD && Robot.elevator.getElevatorPosition() != ElevatorPosition.FIRST) {
                move = new ElevatorMove(ElevatorPosition.FIRST, 0.8);
                move.start();
                hasMovedUp = true;
            }
        } else {
            hasMovedUp = false;
        }

        if (Math.abs(angle) < thresholdAngle) { // keep going in this direction
            if (distance == 0) {
                Robot.driveSubsystem.drive(0.1, 0.1);
            } else {
                Robot.driveSubsystem.drive(normalPower, normalPower);
            }
        } else {
            if (angle > thresholdAngle) { // Robot is to the left of the target
                Robot.driveSubsystem.drive(turnPower + addedPower, addedPower);
            } else { // Robot is to the right of the target
                Robot.driveSubsystem.drive(addedPower, turnPower + addedPower);
            }
        }
        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.driveSubsystem.stopMotors();
    }

    @Override
    protected void interrupted() {
        Robot.driveSubsystem.stopMotors();
    }
}
