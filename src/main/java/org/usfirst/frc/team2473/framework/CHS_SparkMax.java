package org.usfirst.frc.team2473.framework;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team2473.robot.RobotMap;

public class CHS_SparkMax {
	private CANSparkMax spark;
	private CANEncoder encoder;

	private double lastEncoderValue = 0;

	public CHS_SparkMax(int port, MotorType motorType) {
		spark = new CANSparkMax(port, motorType);
		encoder = spark.getEncoder();
	}

	public CANSparkMax getSparkMaxObject() {
		return spark;
	}

	public void set(double power) {
		spark.set(power);
	}

	public void stop() {
		spark.set(0);
	}

	public double getEncoderPosition() {
        double change = 1;
        if (getSparkMaxObject().getDeviceId() == RobotMap.SPARK_L) change = -1;
		if (encoder.getPosition() == 0) {
			return change * lastEncoderValue;
		} else {
			lastEncoderValue = encoder.getPosition();
			return change * lastEncoderValue;
		}
	}

	public double getEncoderPositionUnmodified() {
		return encoder.getPosition();
	}

	public double getEncoderVelocity() {
		return encoder.getVelocity();
	}
}