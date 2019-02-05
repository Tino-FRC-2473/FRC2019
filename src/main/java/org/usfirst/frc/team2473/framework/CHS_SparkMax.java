package org.usfirst.frc.team2473.framework;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team2473.robot.RobotMap;

public class CHS_SparkMax {
	private CANSparkMax spark;
	private CANEncoder encoder;

	/* At times, the encoders randomly return zero by mistake */
	private double lastEncoderValue = 0;
	
	/* Used to calculate the encoder position after a reset */
	private double encoderOffset = 0;

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
		double position = encoder.getPosition();
		if (position != 0) {
			lastEncoderValue = position - encoderOffset;
		}
		
		return lastEncoderValue;
	}

	//Emily is confused as to the purpose of this method
	// public double getEncoderPositionUnmodified() {
	// 	return encoder.getPosition();
	// }

	/* Note that this may return 0 when it should NOT be zero */
	public double getRawEncoderPosition() {
	 	return encoder.getPosition();
	}

	public double getEncoderVelocity() {
		return encoder.getVelocity();
	}
	/**
	 * "Resets" the encoder by establishing an offset value that 
	 * the getEncoderPosition() method would be based on
	 * 
	 * @return the offset value
	 */
	public double resetEncoder(){
		encoderOffset = encoder.getPosition();
		return encoderOffset;
	}
}