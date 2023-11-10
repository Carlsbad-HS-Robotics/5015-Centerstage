package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm extends SubsystemBase {
    private ServoEx wrist;
    private ServoEx elbow0;
    private ServoEx elbow1;
    private ServoEx claw;
    private int low;
    private int high;
    private boolean isLow;
    private boolean isHolding;
    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 359;
    public Arm(final HardwareMap hMap, String name){
        elbow0 = new SimpleServo(hMap, "elbow0",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        elbow1 = new SimpleServo(hMap, "elbow1",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        claw = new SimpleServo(hMap, "claw",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        wrist = new SimpleServo(hMap, "wrist",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        elbow1.setInverted(true);

    }
    public void low(){
        elbow0.turnToAngle(0);
        elbow1.turnToAngle(0);

            isLow = true;


    }
    public void grab(){
        claw.turnToAngle(90);
        isHolding = true;
    }
    public void release(){
         claw.turnToAngle(0);
         isHolding = false;
    }
    public void high(){
        elbow0.turnToAngle(170);
        elbow1.turnToAngle(170);
            isLow = false;

    }
    public boolean getHoldState(){
        return isHolding;
    }
    public boolean getArmState(){
        return isLow;
    }

}
