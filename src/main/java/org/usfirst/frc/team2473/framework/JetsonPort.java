package org.usfirst.frc.team2473.framework;

import org.usfirst.frc.team2473.framework.Logging;
import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.SerialPort;

public class JetsonPort extends SerialPort {

    private static final String START = "S";
    private static final String END = "E";

    private int fVisionAngle = 0;
    private int fVisionDistance = 0;
    private int bVisionAngle = 0;
    private int bVisionDistance = 0;
     
    private boolean firstStart = false;
    private String buffer = "";
    
    public JetsonPort(int baudrate, Port port) {
        super(baudrate, port);
    }

    public void updateVisionValues() {
        try {
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
                    if (data.length() == 12) {
                        fVisionAngle = Integer.parseInt(data.substring(0, 3));
                        fVisionDistance = Integer.parseInt(data.substring(3, 6));
                        bVisionAngle = Integer.parseInt(data.substring(6, 9));
                        bVisionDistance = Integer.parseInt(data.substring(9));

                        buffer = buffer.substring(buffer.indexOf(END)+1);
                    }
                    
                }
            }
        } catch (Exception e) {
            Logger.getInstance().logError(e.getClass());
            RobotMap.CV_RUNNING = false;
        }
        
    }

    public void printVisionAngles() {
        Logger.getInstance().logInfo(String.format("FRONT: Angle %3d Distance %3d        BACK:  Angle %3d Distance %3d", fVisionAngle, fVisionDistance, bVisionAngle, bVisionDistance));
    }

    public double getVisionAngle() {
        return RobotMap.RUNNING_FORWARD ? fVisionAngle : bVisionAngle;
    }

    public double getVisionDistance() {
        return RobotMap.RUNNING_FORWARD ? fVisionDistance : bVisionDistance;
    }
}