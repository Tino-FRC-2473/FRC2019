package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {

    double normalPower = 0.3;
    double turnPower = 0.08;
    double lowPower = 0.1;
    private int angle = 0;
    public int distance = 0;
    public static int x = 0;

    private boolean canSwitchTargets = true;

    public static boolean isRunning = false;

    public static boolean isCurrentlyTurning = true;

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
        isCurrentlyTurning = true;
    }

    public void calculateTarget() {
        CVData target1 = new CVData(Robot.jetsonPort.getVisionAngle1(), Robot.jetsonPort.getVisionDistance1(), Robot.jetsonPort.getVisionX1(), 1);
        CVData target2 = new CVData(Robot.jetsonPort.getVisionAngle2(), Robot.jetsonPort.getVisionDistance2(), Robot.jetsonPort.getVisionX2(), 2);
        CVData target3 = new CVData(Robot.jetsonPort.getVisionAngle3(), Robot.jetsonPort.getVisionDistance3(), Robot.jetsonPort.getVisionX3(), 3);

        CVData[] targets = new CVData[]{target1, target2, target3};

        if (Math.abs(Robot.oi.getWheel().getX()) < 0.1) {
            canSwitchTargets = true;
        }

        if (angle == -99 && distance == -99) {
            Arrays.sort(targets);
            int i = 0;
            while (i < 3 && targets[i].angle == -99 && targets[i].distance == -99) i++;
            
            if (i == 3) {
                reset();
            } else {
                angle = targets[i].angle;
                distance = targets[i].distance;
                x = targets[i].x;
            }
        } else if (Robot.oi.getWheel().getX() > 0.2 && canSwitchTargets) {
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
        if (angle != -99) angle += 3;


        /*

        --------------
            TODO    
        --------------

        I don't think we need the first two clauses in the below if statement,
        since during CV the arm is always deployed out, and I don't think
        we need a distance check.

        */

        // Modify angle if we are scoring a hatch
        if (RobotMap.SCORING_HATCH && distance > 35 && angle != -99) {
            double x1 = 29.75;
            double x2 = distance - x1;
            double h = distance * Math.tan(Math.toRadians(angle));

            angle = (int) Math.toDegrees(Math.atan(h/x2));
        }


        double thresholdAngle = 1;

        System.out.println(distance);
        
        if (Robot.elevator.isMoving() || Robot.arm.isMoving()) {
            Robot.driveSubsystem.stopMotors();
        } else if (isCurrentlyTurning) {
            double power = RobotMap.MINIMUM_DRIVE_TURN_POWER + 0.05*Math.abs(angle);
            if (angle > thresholdAngle) {
                Robot.driveSubsystem.drive(power, -power);
            } else if (angle < -thresholdAngle) {
                Robot.driveSubsystem.drive(-power, power);
            } else {
                isCurrentlyTurning = false;
            }
        } else {
            if (angle > 0) {
                Robot.driveSubsystem.drive(normalPower + 0.02, normalPower);
            } else if (angle < 0) {
                Robot.driveSubsystem.drive(normalPower, normalPower + 0.02);
            } else {
                Robot.driveSubsystem.drive(normalPower, normalPower);
            }
        }
        
        
        // if (Robot.elevator.isMoving() || Robot.arm.isMoving()) {
        //     Robot.driveSubsystem.stopMotors();
        // } else if ((TeleopDrive.hasRaised) || distance < thresholdDistance || angle == -99) { // keep going in this direction
        //     Robot.driveSubsystem.drive(0.1, 0.1);
        // } else if (Math.abs(angle) < thresholdAngle) {
        //     Robot.driveSubsystem.drive(normalPower, normalPower);
        // } else {
        //     if (angle > thresholdAngle) { // Robot is to the left of the target
        //         Robot.driveSubsystem.drive(turnPower + lowPower, lowPower);
        //     } else { // Robot is to the right of the target
        //         Robot.driveSubsystem.drive(lowPower, turnPower + lowPower);
        //     }
        // }

        if (angle != -99) angle -=3;
        
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
