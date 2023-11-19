package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
public class Arm{
    private ServoEx elbow0;
    private ServoEx elbow1;
    private ServoEx claw;
    private ServoEx wrist;
    private CRServo hang0;
    private CRServo hang1;
    private Motor vHang;
    private int low;
    private int high;
    private boolean isLow;
    private boolean isHolding;
    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 300;
    public static int lowAngle = 0;
    public static int highAngle = 300;
    public Arm(final HardwareMap hMap){
        elbow0 = new SimpleServo(hMap, "elbow0",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        elbow1 = new SimpleServo(hMap, "elbow1",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        wrist = new SimpleServo(hMap, "wrist",MIN_ANGLE,MAX_ANGLE, AngleUnit.DEGREES);
        claw = new SimpleServo(hMap,"claw",MIN_ANGLE,MAX_ANGLE,AngleUnit.DEGREES);
        vHang = new Motor(hMap,"vHang");
        hang0 = hMap.crservo.get("hang0");
        hang1 = hMap.crservo.get("hang1");
        elbow1.setInverted(true);
        elbow0.setInverted(true);
        hang0.setDirection(DcMotorSimple.Direction.REVERSE);

        //

    }
    public void low(){
        elbow0.setPosition(0);
        elbow1.setPosition(0);
        wrist.setPosition(0.75);
            isLow = true;


    }
    public void grab(){
        claw.turnToAngle(90);
        isHolding = true;
    }
    public void release(){
         claw.turnToAngle(200);
         isHolding = false;
    }
    public void high(){
        elbow0.setPosition(1);
        elbow1.setPosition(1);
            isLow = false;
            wrist.setPosition(0);

    }
    public void drop(){
        elbow0.setPosition(0.65);
        elbow1.setPosition(0.65);
        wrist.setPosition(0);
    }
    public void hangUp(){
        vHang.set(1);
    }
    public void hangDown(){
        vHang.set(-1);
    }
    public void hangServoUp(){
        hang0.setPower(1);
        hang1.setPower(1);
    }
    public void hangServoDown(){
        hang0.setPower(-1);
        hang1.setPower(-1);
    }
    public void hangOff(){
        hang0.setPower(0);
        hang1.setPower(0);
        vHang.set(0);
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
