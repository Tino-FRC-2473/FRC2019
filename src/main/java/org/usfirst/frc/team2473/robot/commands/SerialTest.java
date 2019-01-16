/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot.commands;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2473.robot.Robot;

/**
 * A class that sets the talons to specific powers upon current joystick positions
 */
public class SerialTest extends Command {
    
    private SerialPort port;

	public SerialTest() {
		
	}
    
    @Override
    protected void initialize() {
        port = new SerialPort(9600, SerialPort.Port.kUSB);
    }
	@Override
	protected void execute() {
        port.writeString("Yoot\n");
        System.out.println(port.readString());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}