package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.NanoClock;

@Config
public class Arm{
    private ServoEx elbow0;
    private ServoEx elbow1;
    private ServoEx claw;
    private ServoEx wrist;
    private CRServo hang0;
    private CRServo hang1;
    private Motor vHang;
    private double elbowPosition;
    private int low;
    private int high;
    private boolean isLow;
    private boolean isHolding;
    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 300;
    public static int lowAngle = 0;
    public static int highAngle = 300;
    public NanoClock armClock;
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
        wrist.setInverted(true);
        hang0.setDirection(DcMotorSimple.Direction.REVERSE);
        armClock = new NanoClock();

    }
    public void high(){
        elbowPosition = 0;
        wrist.setPosition(1);
            isLow = true;


    }

    public void setElbowPosition(double a){
        if(elbowPosition <=1 && elbowPosition >= 0){
            elbowPosition = a;
        } else if (elbowPosition > 1){
            elbowPosition = 1;
        } else {
            elbowPosition = 0;
        }
    }
    public void grab(){
        claw.turnToAngle(20);
        isHolding = true;
    }
    public void release(){
         claw.turnToAngle(180);
         isHolding = false;
    }
    public void low(){
        wrist.setPosition(0.43);
        elbowPosition = 1;
            isLow = false;


    }
    public void drop(){
        setElbowPosition(0.55);
        wrist.setPosition(0.43);
    }
    public void rotateArm(){

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
    public void update(){
        elbow0.setPosition(elbowPosition);
        elbow1.setPosition(elbowPosition);
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
    public double getElbowPosition(){
        return elbowPosition;
    }
}
