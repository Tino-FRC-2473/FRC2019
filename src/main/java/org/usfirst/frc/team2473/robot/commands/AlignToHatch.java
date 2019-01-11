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

import java.io.IOException;
import java.util.Arrays;

import org.usfirst.frc.team2473.framework.Devices;
import org.usfirst.frc.team2473.framework.UtilitySocket;

/**
 * A class that aligns the robot to the hatch based on the angle provided by CV
 */
public class AlignToHatch extends Command {
	
	private double normalPower = 0.25;
    private double addedPower = 0.15;
    private UtilitySocket cvSocket;
    private double cvAngle = 0;
    private double cvDistance = 0;

	public AlignToHatch() {
        requires(Robot.driveSubsystem);
        initializeCVSocket();
	}
 
	@Override
	protected void execute() {
		move();
    }
    
    public void move() {
        fetchCVData();
        double angle = getCVAngle();
        double distance = getCVDistance();

        double adjustAddedPower = addedPower;
        double adjustNormalPower = normalPower;
        if (distance < 10 && Math.abs(angle) > 10) {
            adjustAddedPower *= 1.5;
            adjustNormalPower = 0.1;
        }

        System.out.println("Angle: " + angle);
        System.out.println("Distance: " + distance);
		if (Math.abs(angle) < 1) { // keep going in this direction
			Robot.driveSubsystem.drive(adjustNormalPower, adjustNormalPower, adjustNormalPower, adjustNormalPower);
		} else if (angle > 1) { // Robot is to the left of the target
			Robot.driveSubsystem.drive(adjustNormalPower + adjustAddedPower, adjustNormalPower + adjustAddedPower, adjustNormalPower, adjustNormalPower);
		} else { // Robot is to the right of the target
			Robot.driveSubsystem.drive(adjustNormalPower, adjustNormalPower, adjustNormalPower + adjustAddedPower, adjustNormalPower + adjustAddedPower);
		}
    }

	@Override
	protected boolean isFinished() {
		return false;
    }

    @Override
    protected void interrupted() {
        closeSocketConnection();
    }
    
    public void initializeCVSocket() {
        if (cvSocket == null) {
            try {
                System.out.println("Creating socket for CV");
                cvSocket = new UtilitySocket(RobotMap.JETSON_IP, RobotMap.JETSON_PORT);
                System.out.println("Utility socket created!");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void fetchCVData() throws NullPointerException {
        if (cvSocket != null) {
            System.out.println("Getting CV Data...");
            String data = cvSocket.getLine();
            System.out.println("Received CV Data");
            if (data != null) {
                String[] str = data.split(" ");
                String angleStr = str[0];
                String distanceStr = str[1];
                System.out.println(Arrays.toString(str));
                cvAngle = Double.parseDouble(angleStr);
                cvDistance = Double.parseDouble(distanceStr);
            }
        }
        else {
            throw new NullPointerException("CV Socket not initialized");
        }
    }
    
    public double getCVAngle() {
        return cvAngle;
    }

    public double getCVDistance() {
        return cvDistance;
    }

    public void closeSocketConnection() {
        try {
            if (cvSocket != null) {
                cvSocket.close();
                cvSocket = null;
                cvAngle = 0;
                cvDistance = 0;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        
    }
}