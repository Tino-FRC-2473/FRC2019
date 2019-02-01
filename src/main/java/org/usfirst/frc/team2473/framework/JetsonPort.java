package org.usfirst.frc.team2473.framework;

import edu.wpi.first.wpilibj.SerialPort;

public class JetsonPort 
//extends SerialPort 
{

    private double visionAngle = 0;
    private double visionDistance = 0;
    private int numCalls = 0;
    private int saveSum = 0;

    private static JetsonPort theInstance;

    static {
		theInstance = new JetsonPort();
		//theInstance = new JetsonPort(9600, Port.kUSB);
    }
    
    // private JetsonPort(int baudrate, Port port) {
    //     super(baudrate, port);
    // }

    public static JetsonPort getInstance() { 
		return theInstance;
	}

    public void updateVisionValues() {}

    /*public void updateVisionValues() {
        String recieve = readString();
        if(recieve.length() != 0 && 
            recieve.contains(" ") &&
            recieve.indexOf(" ") != recieve.length()-1) {
            try {
                String[] split = recieve.split(" ");

                String visionAngleStr = split[0];
                String visionDistanceStr = split[1];

                //System.out.println("angle: ///" + visionAngleStr + "/// distance: ///" + visionDistanceStr + "///");

                if (visionAngleStr.charAt(0) != 'J' || visionDistanceStr.charAt(visionDistanceStr.length()-1) != 'J') {
                    //System.out.println("No J");
                    return;
                }
                
            
                double ang = Double.parseDouble(visionAngleStr.substring(1));
                if (ang != -2473) {
                    visionAngle = ang;
                    visionDistance = Double.parseDouble(visionDistanceStr.substring(0, visionDistanceStr.length() - 1));
                }
            } catch (Exception e) {
                
            }
        }
        
    }*/

    public double getVisionAngle() {
        return Math.random()*180 - 90;
    }
    // public double getVisionAngle() {
    //     return visionAngle;
    // }

    public double getVisionDistance() {
        return Math.random()*60;
    }
    // public double getVisionDistance() {
    //     return visionDistance;
    // }
}