package org.usfirst.frc.team2473.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Roller extends Subsystem {

    private WPI_TalonSRX talon; 

    private static Roller instance;
	
	static {
		instance = new Roller();
    }
    
    public static Roller getInstance() {
		return instance;
	}
    
    
	private Roller() {
		talon = new WPI_TalonSRX(RobotMap.TALON_ROLLER);
        
    }

    public void set(double speed) {
        if (Robot.arm.distanceSensor.getVoltage() > 1.9 && speed < 0) {
            talon.set(0);
        } else {
            talon.set(speed);
        }
    }

    @Override
	public void initDefaultCommand() {}
}