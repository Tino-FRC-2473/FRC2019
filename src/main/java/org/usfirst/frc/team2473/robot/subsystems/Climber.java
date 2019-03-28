package org.usfirst.frc.team2473.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team2473.framework.State;
import org.usfirst.frc.team2473.robot.Robot;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

    private WPI_TalonSRX frontTalon;
    private WPI_TalonSRX backTalon; 

    private static Climber instance;
	
	static {
		instance = new Climber();
    }
    
    public static Climber getInstance() {
		return instance;
	}
    
    
	private Climber() {
		frontTalon = new WPI_TalonSRX(RobotMap.TALON_FRONT_CLIMB);
		backTalon = new WPI_TalonSRX(RobotMap.TALON_BACK_CLIMB);
    }

    public void setFrontDeploy() {
        frontTalon.set(1);
    }

    public void setFrontRetract() {
        frontTalon.set(-1);
    }

    public void setFrontStop() {
        frontTalon.set(0);
    }

    public void setBackDeploy() {
        backTalon.set(-1);
    }

    public void setBackRetract() {
        backTalon.set(1);
    }

    public void setBackStop() {
        backTalon.set(0);
    }

    @Override
	public void initDefaultCommand() {}
}