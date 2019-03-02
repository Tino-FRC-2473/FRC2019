/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2473.robot;

public class RobotMap {
	public static final int TALON_BL = 3;
	public static final int TALON_BR = 6;
	public static final int TALON_FL = 2;
	public static final int TALON_FR = 7;

	public static final int SPARK_L = 3;
    public static final int SPARK_R = 4;
    
    public static final int TALON_ELEVATOR = 11;
    public static final int TALON_ARM = 12;
    public static final int TALON_ROLLER = 13;
    
    public static final int K_ENCODER_ARM_THRESHOLD = 12;
    
    public static final int K_ENCODER_ELEVATOR_THRESHOLD = 12;
    public static final int ELEVATOR_MIN_TICKS = 50;
    
	public static double K_TICKS_PER_INCH = 0.5398;
	public static double K_ENCODER_THRESHOLD = 4 * K_TICKS_PER_INCH;
	
	public static double K_START_STALL_POWER = 0.25;
	public static double K_RUNNING_STALL_POWER = 0.2;
	public static double K_OPPOSITE_POWER = 0.2;
	public static double K_TURN = 0.95;

    public static boolean SCORING_HATCH = true;
    public static boolean CAMERAS_SWITCHED = false;
    public static boolean CV_RUNNING = false;
	
	public static double DEADBAND_MINIMUM_POWER = 0.2;
	public static double DEADBAND_MINIMUM_TURN = 0.1;


    public static final int CV_LIGHT = 1;
    public static final String JETSON_IP = "10.24.73.19";
    public static final int JETSON_PORT = 5801;
}
