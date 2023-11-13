package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm{
    private ServoEx elbow0;
    private ServoEx elbow1;
    private ServoEx claw;
    private ServoEx wrist;
    private int low;
    private int high;
    private boolean isLow;
    private boolean isHolding;
    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 300;
    public Arm(final HardwareMap hMap){
        elbow0 = new SimpleServo(hMap, "elbow0",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        elbow1 = new SimpleServo(hMap, "elbow1",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        wrist = new SimpleServo(hMap, "wrist",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        claw = new SimpleServo(hMap, "claw",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);

        //

    }
    public void low(){
        elbow0.turnToAngle(00);
        elbow1.turnToAngle(00);
        wrist.turnToAngle(0);
            isLow = true;


    }
    public void grab(){
        claw.turnToAngle(90);
        isHolding = true;
    }
    public void release(){
         claw.turnToAngle(300);
         isHolding = false;
    }
    public void high(){
        elbow0.turnToAngle(180);
        elbow1.turnToAngle(180);
            isLow = false;
            wrist.turnToAngle(33);

    }
    public boolean getHoldState(){
        return isHolding;
    }
    public boolean getArmState(){
        return isLow;
    }
    public double getServoPos(ServoEx servo){
        return servo.getAngle();
    }
    public double getElbowAngle(){
        return getServoPos(elbow0);
    }
    public double getWristAngle(){
        return getServoPos(wrist);
    }

}
