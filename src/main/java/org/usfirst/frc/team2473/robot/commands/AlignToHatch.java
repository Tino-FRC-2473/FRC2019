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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.usfirst.frc.team2473.framework.JetsonPort;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {

    double normalPower = 0.2;
    double turnPower = 0.08;
    double lowPower = 0.1;
    private int angle = 0;
    public int distance = 0;
    public static int x = 0;

    // private boolean hasMovedUp;
    // private boolean hasMovedBase;

    // private ElevatorMove move = null;
    private boolean canSwitchTargets = true;
    private boolean hasBeenInThresholdDistance = false;

    public static boolean isRunning = false;

    public AlignToHatch() {
        requires(Robot.driveSubsystem);
    }

    @Override
    protected void execute() {
        move();
    }

    public void reset() {
        angle = -99;
        distance = -99;
        isRunning = false;
        hasBeenInThresholdDistance = false;
    }

    public void calculateTarget() {
        CVData target1 = new CVData(Robot.jetsonPort.getVisionAngle1(), Robot.jetsonPort.getVisionDistance1(), Robot.jetsonPort.getVisionX1(), 1);
        CVData target2 = new CVData(Robot.jetsonPort.getVisionAngle2(), Robot.jetsonPort.getVisionDistance2(), Robot.jetsonPort.getVisionX2(), 2);
        CVData target3 = new CVData(Robot.jetsonPort.getVisionAngle3(), Robot.jetsonPort.getVisionDistance3(), Robot.jetsonPort.getVisionX3(), 3);

        CVData[] targets = new CVData[]{target1, target2, target3};

        //System.out.println(Arrays.toString(targets));

        if (Math.abs(Robot.oi.getWheel().getX()) < 0.2) {
            canSwitchTargets = true;
        }

        if (angle == -99 && distance == -99) {
            Arrays.sort(targets);
            //System.out.println("TARGET: " + targets[0]);
            int i = 0;
            while (i < 3 && targets[i].angle == -99 && targets[i].distance == -99) i++;
            
            if (i == 3) {
                reset();
            } else {
                angle = targets[i].angle;
                distance = targets[i].distance;
                x = targets[i].x;
            }
        } else if (Robot.oi.getWheel().getX() > 0.4 && canSwitchTargets) {
            CVData closestRightTarget = null;

            for (CVData target : targets) {
                if (target.x - x > 10) {
                    if (closestRightTarget == null || target.x - x < closestRightTarget.x - x) {
                        closestRightTarget = target;
                    }
                }
            }

            if (closestRightTarget == null) {
                // there was no target to the right
                
            } else {
                angle = closestRightTarget.angle;
                distance = closestRightTarget.distance;
                x = closestRightTarget.x;
            }

            canSwitchTargets = false;
        } else if (Robot.oi.getWheel().getX() < -0.4 && canSwitchTargets) {
            CVData closestLeftTarget = null;

            for (CVData target : targets) {
                if (target.x - x < -10) {
                    if (closestLeftTarget == null || target.x - x > closestLeftTarget.x - x) {
                        closestLeftTarget = target;
                    }
                }
            }

            if (closestLeftTarget == null) {
                // there was no target to the left
                
            } else {
                angle = closestLeftTarget.angle;
                distance = closestLeftTarget.distance;
                x = closestLeftTarget.x;
            }

            canSwitchTargets = false;
        } else {
            CVData delta1 = null;
            CVData delta2 = null;
            CVData delta3 = null;
            
            if (target1.angle != -99 && target1.distance != -99) {
                delta1 = new CVData(target1.angle - angle, target1.distance - distance, target1.x, target1.id);
            }

            if (target2.angle != -99 && target2.distance != -99) {
                delta2 = new CVData(target2.angle - angle, target2.distance - distance, target2.x, target2.id);
            }

            if (target3.angle != -99 && target3.distance != -99) {
                delta3 = new CVData(target3.angle - angle, target3.distance - distance, target3.x, target3.id);
            }



            ArrayList<CVData> deltas = new ArrayList<>();
            if (delta1 != null) deltas.add(delta1);
            if (delta2 != null) deltas.add(delta2);
            if (delta3 != null) deltas.add(delta3);

            Collections.sort(deltas);
            //System.out.println("Deltas: " + deltas);

            CVData closestTarget = null;

            if (deltas.size() != 0) {
                switch (deltas.get(0).id) {
                    case 1:
                        closestTarget = target1;
                        break;
                    case 2:
                        closestTarget = target2;
                        break;
                    case 3:
                        closestTarget = target3;
                        break;
                    default:
                        break;
                }
    
                //System.out.println("TARGET: " + closestTarget);
                angle = closestTarget.angle;
                distance = closestTarget.distance;
                x = closestTarget.x;
            }
        }
    }

    public void move() {
        if (!RobotMap.CV_RUNNING) return;
        
        isRunning = true;

        calculateTarget();

        if (distance == 99) return;

        //System.out.println(distance);

        double thresholdAngle = 2;
        double thresholdDistance = 50;
        // angle = Robot.jetsonPort.getVisionAngle();
        // distance = Robot.jetsonPort.getVisionDistance();

        // temporarily negate angle for moving to targets
        if (!RobotMap.SCORING_HATCH) angle = -angle;

        System.out.println(distance);
        if (Robot.elevator.isMoving() || Robot.arm.isMoving()) {
            Robot.driveSubsystem.stopMotors();
        } else if (hasBeenInThresholdDistance || distance < thresholdDistance || angle == -99) { // keep going in this direction
            hasBeenInThresholdDistance = true;
            Robot.driveSubsystem.drive(0.1, 0.1);
        } else if (Math.abs(angle) < thresholdAngle) {
            Robot.driveSubsystem.drive(normalPower, normalPower);
        } else {
            if (angle > thresholdAngle) { // Robot is to the left of the target
                Robot.driveSubsystem.drive(turnPower + lowPower, lowPower);
            } else { // Robot is to the right of the target
                Robot.driveSubsystem.drive(lowPower, turnPower + lowPower);
            }
        }

        // re-negate angle
        if (!RobotMap.SCORING_HATCH) angle = -angle;
        
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

    private class CVData implements Comparable<CVData> {
        int angle;
        int distance;
        int x;
        int id;

        public CVData(int angle, int distance, int x, int id) {
            this.angle = angle;
            this.distance = distance;
            this.x = x;
            this.id = id;
        }

        @Override
        public int compareTo(CVData o) {
            if (angle != o.angle) {
                return Integer.compare(Math.abs(angle), Math.abs(o.angle));
            } else {
                return Integer.compare(distance, o.distance);
            }
        }

        @Override
        public String toString() {
            return "CVData[" + angle + " " + distance + " " + x + " " + id + "]";
        }


    }
}
