package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo  ;
import com.qualcomm.robotcore.hardware.DcMotor  ;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo    ;

import TeamCode.AbstractClasses.AbstractRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Sebby;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    Sebby robot;
    public DcMotor intakeLeft, intakeRight;
    
    public Intake(AbstractRobot robot, String iwl, String iwr) {
        super (robot);
        this.robot = (Sebby) robot;

        intakeLeft  = robot.hardwareMap.get(DcMotor.class, iwl);
        intakeRight = robot.hardwareMap.get(DcMotor.class, iwr);
    }

    @Override
    public void init(){
        intakeLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //FIXME
        //intakeRight.setDirection.REVERSE() whatever the code is.. just reverse it
    }

    @Override
    public void start() {}

    @Override
    public void driverLoop() {
        //FIXME (my error things arnt working)
        //TODO : update for propper robot things + telem
        intakeRight.setPower(robot.gamepad1.rightTrigger);
        intakeLeft .setPower(robot.gamepad1.rightTrigger);
    }

    @Override
    public void stop() {}
}