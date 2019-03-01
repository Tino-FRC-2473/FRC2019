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

    private int bVisionAngle1 = 0;
    private int bVisionDistance1 = 0;
    private int bVisionX1 = 0;

    private int bVisionAngle2 = 0;
    private int bVisionDistance2 = 0;
    private int bVisionX2 = 0;

    private int bVisionAngle3 = 0;
    private int bVisionDistance3 = 0;
    private int bVisionX3 = 0;
     
    private boolean firstStart = true;
    private String buffer = "";
    private int blankCount = 0;
    
    public JetsonPort(int baudrate, Port port) {
        super(baudrate, port);
    }

    /*

    SAAADDDXXXAAADDDXXXAAADDDXXXAAADDDXXXAAADDDXXXAAADDDXXXE

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
                    if (data.length() == 54) {
                        fVisionAngle1 = Integer.parseInt(data.substring(0, 3));
                        fVisionDistance1 = Integer.parseInt(data.substring(3, 6));
                        fVisionX1 = Integer.parseInt(data.substring(6, 9));

                        fVisionAngle2 = Integer.parseInt(data.substring(9, 12));
                        fVisionDistance2 = Integer.parseInt(data.substring(12, 15));
                        fVisionX2 = Integer.parseInt(data.substring(15, 18));

                        fVisionAngle3 = Integer.parseInt(data.substring(18, 21));
                        fVisionDistance3 = Integer.parseInt(data.substring(21, 24));
                        fVisionX3 = Integer.parseInt(data.substring(24, 27));

                        bVisionAngle1 = Integer.parseInt(data.substring(27, 30));
                        bVisionDistance1 = Integer.parseInt(data.substring(30, 33));
                        bVisionX1 = Integer.parseInt(data.substring(33, 36));

                        bVisionAngle2 = Integer.parseInt(data.substring(36, 39));
                        bVisionDistance2 = Integer.parseInt(data.substring(39, 42));
                        bVisionX2 = Integer.parseInt(data.substring(42, 45));

                        bVisionAngle3 = Integer.parseInt(data.substring(45, 48));
                        bVisionDistance3 = Integer.parseInt(data.substring(48, 51));
                        bVisionX3 = Integer.parseInt(data.substring(51, 54));

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
        System.out.println(String.format("FRONT: Angle %3d Distance %3d        BACK:  Angle %3d Distance %3d\nFRONT: Angle %3d Distance %3d        BACK:  Angle %3d Distance %3d\nFRONT: Angle %3d Distance %3d        BACK:  Angle %3d Distance %3d\n", fVisionAngle1, fVisionDistance1, bVisionAngle1, bVisionDistance1, fVisionAngle2, fVisionDistance2, bVisionAngle2, bVisionDistance2, fVisionAngle3, fVisionDistance3, bVisionAngle3, bVisionDistance3));
    }

    public int getVisionAngle1() {
        return RobotMap.RUNNING_FORWARD ? fVisionAngle1 : bVisionAngle1;
    }

    public int getVisionAngle2() {
        return RobotMap.RUNNING_FORWARD ? fVisionAngle2 : bVisionAngle2;
    }

    public int getVisionAngle3() {
        return RobotMap.RUNNING_FORWARD ? fVisionAngle3 : bVisionAngle3;
    }

    public int getVisionDistance1() {
        return RobotMap.RUNNING_FORWARD ? fVisionDistance1 : bVisionDistance1;
    }

    public int getVisionDistance2() {
        return RobotMap.RUNNING_FORWARD ? fVisionDistance2 : bVisionDistance2;
    }

    public int getVisionDistance3() {
        return RobotMap.RUNNING_FORWARD ? fVisionDistance3 : bVisionDistance3;
    }

    public int getVisionX1() {
        return RobotMap.RUNNING_FORWARD ? fVisionX1 : bVisionX1;
    }

    public int getVisionX2() {
        return RobotMap.RUNNING_FORWARD ? fVisionX2 : bVisionX2;
    }

    public int getVisionX3() {
        return RobotMap.RUNNING_FORWARD ? fVisionX3 : bVisionX3;
    }
}