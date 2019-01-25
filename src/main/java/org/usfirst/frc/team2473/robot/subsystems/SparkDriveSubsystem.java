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
        initLookupTable();
	}
	
	public static SparkDriveSubsystem getInstance() {
		return theInstance;
    }
    
    private void initLookupTable() {
		leftTable.put(0.2, 1.15);
		leftTable.put(0.3, 1.15);
		leftTable.put(0.4, 1.1);
		leftTable.put(0.5, 1.055146);
		leftTable.put(0.6, 1.052553);
		leftTable.put(0.7, 1.061291);
		leftTable.put(0.8, 1.088878);
		
		rightTable.put(0.2, 1.0);
		rightTable.put(0.3, 1.0);
		rightTable.put(0.4, 1.0);
		rightTable.put(0.5, 1.0); // was 1.025404
		rightTable.put(0.6, 1.0);
		rightTable.put(0.7, 1.0);
		rightTable.put(0.8, 1.0);
		
		minTestedPower = Collections.min(leftTable.keySet());
		maxTestedPower = Collections.max(leftTable.keySet());
	}
	
	/**
	 * Transforms raw motor powers using an experimental lookup table to account for the difference in motor outputs.
	 * @param power		raw motor power
	 * @param motor		motor for which the power is being set	
	 * @return converted motor power
	 */
	public double convertPower(double power, CHS_SparkMax motor) {
		// determine which lookup table to use
		boolean isLeft = motor.equals(leftSpark) ? true : false;
		HashMap<Double, Double> tempTable = isLeft ? leftTable : rightTable;
		
		if (power < minTestedPower) {
			return power/tempTable.get(minTestedPower);
		} else if (power > maxTestedPower) {
			return power/tempTable.get(maxTestedPower);
		} else {
			
			ArrayList<Double> powers = new ArrayList<>(tempTable.keySet());
			Collections.sort(powers);
			
			if (powers.contains(power)) { // the input power is one of the powers in the lookup table
				double newPower = power/tempTable.get(power);
				return newPower;
			}
			
			// linearize between the two values around the power
			int i;
			for (i = powers.size() - 1; powers.get(i) > power; i--);
		
			double lowerNearestPower = powers.get(i); //the largest power value that is lower than the power input in the lookup table
			double higherNearestPower = powers.get(i+1); //the smallest power value that is greater than the power input in the lookup table
			
			double calibrationRatioOfLowerPower = tempTable.get(lowerNearestPower); 
			double calibrationRatioOfHigherPower = tempTable.get(higherNearestPower); 	
			
			double slope = (calibrationRatioOfHigherPower-calibrationRatioOfLowerPower) / (higherNearestPower-lowerNearestPower);
			
			double deltaPower = power - lowerNearestPower; //the change of lookup ratio between the two bounds
			
			double powerCalibration = calibrationRatioOfLowerPower + slope * deltaPower;
			
			return power/powerCalibration;
		}
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

	public void resetEncoder() {
		// reset encoder, currently don't know how
	}

	public double getEncoderTicks(int id) {
		if (id == RobotMap.SPARK_L) return leftSpark.getEncoderPosition();
		else if (id == RobotMap.SPARK_R) return rightSpark.getEncoderPosition();
		throw new IllegalArgumentException("Invalid Spark Max ID");
	}

	public void printEncoders() {
		System.out.println(String.format("LEFT:  Position: %15.3f   |   Velocity: %15.3f", leftSpark.getEncoderPositionUnmodified(), leftSpark.getEncoderVelocity()));
		System.out.println(String.format("RIGHT: Position: %15.3f   |   Velocity: %15.3f", rightSpark.getEncoderPositionUnmodified(), rightSpark.getEncoderVelocity()));
		
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
