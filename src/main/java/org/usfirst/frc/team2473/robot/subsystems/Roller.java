package org.usfirst.frc.team2473.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team2473.framework.CHS_SparkMax;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Roller extends Subsystem {

    private CHS_SparkMax spark; 

    private static Roller instance;
	
	static {
		instance = new Roller();
    }
    
    public static Roller getInstance() {
		return instance;
	}
    
    
	private Roller() {
		spark = new CHS_SparkMax(RobotMap.TALON_ROLLER, MotorType.kBrushless);
        
    }

    public void set(double speed) {
        spark.set(speed);
    }

    @Override
	public void initDefaultCommand() {}
}