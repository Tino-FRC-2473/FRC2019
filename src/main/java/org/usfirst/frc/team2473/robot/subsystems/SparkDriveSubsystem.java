/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc.team2473.robot.RobotMap;
import org.usfirst.frc.team2473.framework.CHS_SparkMax;
import org.usfirst.frc.team2473.framework.Devices;

public class SparkDriveSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	CHS_SparkMax leftSpark;
	CHS_SparkMax rightSpark;
    DifferentialDrive drive;
    
    /**
	 * minimum power for which lookup table values have been determined
	 */
	private double minTestedPower;
	
	/**
	 * maximum power for which lookup table values have been determined
	 */
	private double maxTestedPower;

    private HashMap<Double, Double> leftTable = new HashMap<>();
    private HashMap<Double, Double> rightTable = new HashMap<>();
    
	private static SparkDriveSubsystem theInstance;

	static {
		theInstance = new SparkDriveSubsystem();
	}

	private SparkDriveSubsystem() {
		leftSpark = new CHS_SparkMax(RobotMap.SPARK_L, MotorType.kBrushless);
		rightSpark = new CHS_SparkMax(RobotMap.SPARK_R, MotorType.kBrushless);
        drive = new DifferentialDrive(leftSpark.getSparkMaxObject(), rightSpark.getSparkMaxObject());
	}
	
	public static SparkDriveSubsystem getInstance() {
		return theInstance;
    }
	
	public void teleopDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}

	public void drive(double left, double right) {
		leftSpark.set(-left);
		rightSpark.set(right);
	}

	public void stopMotors() {
		leftSpark.set(0);
		rightSpark.set(0);
	}

	public void resetEncoders() {
		// reset encoder, currently don't know how
	}

	public double getEncoderTicks(int id) {
		if (id == RobotMap.SPARK_L) return -leftSpark.getEncoderPosition();
		else if (id == RobotMap.SPARK_R) return rightSpark.getEncoderPosition();
		throw new IllegalArgumentException("Invalid Spark MAX ID");
	}

	public void printEncoders() {
		System.out.println(String.format("LEFT:  Position: %15.3f   |   Velocity: %15.3f", leftSpark.getEncoderPosition(), leftSpark.getEncoderVelocity()));
		System.out.println(String.format("RIGHT: Position: %15.3f   |   Velocity: %15.3f", rightSpark.getEncoderPosition(), rightSpark.getEncoderVelocity()));
		
	}

	public double encoderDifference() {
		return getEncoderTicks(RobotMap.SPARK_L) - getEncoderTicks(RobotMap.SPARK_R);
	}

	public double getGyroAngle() {
		return Devices.getInstance().getNavXGyro().getAngle();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
