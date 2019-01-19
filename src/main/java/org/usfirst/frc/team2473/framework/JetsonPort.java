package org.usfirst.frc.team2473.framework;

import edu.wpi.first.wpilibj.SerialPort;

public class JetsonPort extends SerialPort {

    private double visionAngle = 0;
    private double visionDistance = 0;

    private static JetsonPort theInstance;

    static {
		theInstance = new JetsonPort(9600, Port.kUSB);
    }
    
    public JetsonPort(int baudrate, Port port) {
        super(baudrate, port);
    }

    public static JetsonPort getInstance() { 
		return theInstance;
	}

    public void updateVisionValues() {
        String recieve = readString();
        if(recieve.length() != 0 && 
            recieve.contains(" ") &&
            recieve.indexOf(" ") != recieve.length()-1) {
            try {
                String[] split = recieve.split(" ");
                visionAngle = Double.parseDouble(split[0]);
                visionDistance = Double.parseDouble(split[1]);
            } catch (Exception e) {}
        }
        
    }

    public double getVisionAngle() {
        return visionAngle;
    }

    public double getVisionDistance() {
        return visionDistance;
    }
}