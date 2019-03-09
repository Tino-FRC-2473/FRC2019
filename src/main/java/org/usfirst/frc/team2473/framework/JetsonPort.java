package org.usfirst.frc.team2473.framework;

import java.awt.Robot;

import org.usfirst.frc.team2473.robot.RobotMap;

import edu.wpi.first.wpilibj.SerialPort;

public class JetsonPort extends SerialPort {

    private static final String START = "S";
    private static final String END = "E";

    private int fVisionAngle1 = 0;
    private int fVisionDistance1 = 0;
    private int fVisionX1 = 0;

    private int fVisionAngle2 = 0;
    private int fVisionDistance2 = 0;
    private int fVisionX2 = 0;

    private int fVisionAngle3 = 0;
    private int fVisionDistance3 = 0;
    private int fVisionX3 = 0;
     
    private boolean firstStart = true;
    private String buffer = "";
    private int blankCount = 0;
    
    public JetsonPort(int baudrate, Port port) {
        super(baudrate, port);
    }

    /*

    SAAADDDXXXAAADDDXXXAAADDDXXXE

    */

    public void updateVisionValues() {
        try {
            String receive = readString();
            if (receive.length() == 0) {
                blankCount++;
            } else {
                blankCount = 0;
                if (!RobotMap.CV_RUNNING) {
                    RobotMap.CV_RUNNING = true;
                    System.out.println("---CV RESTARTED---");
                }
            }
            if (blankCount > 20) {
                System.out.println("---CV STOPPED---");
                RobotMap.CV_RUNNING = false;
                return;
            }
            // System.out.println(blankCount);
            if (firstStart) {
                if (receive.contains(START)) {
                    firstStart = false;
                    receive = receive.substring(receive.indexOf(START));
                }
            }

            if (!firstStart) {
                buffer += receive; 
                if (buffer.contains(END)) {
                    String data = buffer.substring(buffer.indexOf(START)+1, buffer.indexOf(END));
                    if (data.length() == 27) {
                        fVisionAngle1 = Integer.parseInt(data.substring(0, 3));
                        fVisionDistance1 = Integer.parseInt(data.substring(3, 6));
                        fVisionX1 = Integer.parseInt(data.substring(6, 9));

                        fVisionAngle2 = Integer.parseInt(data.substring(9, 12));
                        fVisionDistance2 = Integer.parseInt(data.substring(12, 15));
                        fVisionX2 = Integer.parseInt(data.substring(15, 18));

                        fVisionAngle3 = Integer.parseInt(data.substring(18, 21));
                        fVisionDistance3 = Integer.parseInt(data.substring(21, 24));
                        fVisionX3 = Integer.parseInt(data.substring(24, 27));

                        buffer = buffer.substring(buffer.indexOf(END)+1);
                    }
                    
                }
            }
        } catch (Exception e) {
            System.out.println("ERROR: " + e.getClass());
            RobotMap.CV_RUNNING = false;
        }
        
    }

    public void printVisionAngles() {
        System.out.println(String.format("FRONT: Angle %3d Distance %3d \nFRONT: Angle %3d Distance %3d \nFRONT: Angle %3d Distance %3d \n", fVisionAngle1, fVisionDistance1, fVisionAngle2, fVisionDistance2, fVisionAngle3, fVisionDistance3));
    }

    public int getVisionAngle1() {
        return fVisionAngle1;
    }

    public int getVisionAngle2() {
        return fVisionAngle2;
    }

    public int getVisionAngle3() {
        return fVisionAngle3;
    }

    public int getVisionDistance1() {
        return fVisionDistance1;
    }

    public int getVisionDistance2() {
        return fVisionDistance2;
    }

    public int getVisionDistance3() {
        return fVisionDistance3;
    }

    public int getVisionX1() {
        return fVisionX1;
    }

    public int getVisionX2() {
        return fVisionX2;
    }

    public int getVisionX3() {
        return fVisionX3;
    }
}