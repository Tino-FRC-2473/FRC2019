package org.usfirst.frc.team2473.framework;

import edu.wpi.first.wpilibj.SerialPort;

public class JetsonPort extends SerialPort {

    private static final String START = "S";
    private static final String END = "E";

    private int visionAngle = 0;
    private int visionDistance = 0;
    private boolean firstStart = false;
    private String buffer = "";

    private static JetsonPort theInstance;

    static {
		theInstance = new JetsonPort(9600, Port.kUSB);
    }
    
    private JetsonPort(int baudrate, Port port) {
        super(baudrate, port);
    }

    public static JetsonPort getInstance() { 
		return theInstance;
	}

    public void updateVisionValues() {
        String receive = readString();
        if (!firstStart) {
            if (receive.contains(START)) {
                firstStart = true;
                receive = receive.substring(receive.indexOf(START));
            }
        }
        if (firstStart) {
            buffer += receive; 
            if (buffer.contains(END)) {
                String data = buffer.substring(buffer.indexOf(START)+1, buffer.indexOf(END));
                if (data.length() == 6) {
                    visionAngle = Integer.parseInt(data.substring(0, 3));
                    visionDistance = Integer.parseInt(data.substring(3));

                    buffer = buffer.substring(buffer.indexOf(END)+1);
                }
                
            }
        }
    }


    public double getVisionAngle() {
        return visionAngle;
    }

    public double getVisionDistance() {
        return visionDistance;
    }
}