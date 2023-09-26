package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    //Define the motors for the slides
    public DcMotor slide1 = null;
    public DcMotor slide2 = null;

    public Arm (HardwareMap hardwareMap) {
        slide1 = hardwareMap.dcMotor.get("slide1");
        slide2 = hardwareMap.dcMotor.get("slide2");
    }

    public void Extend() {

    }

    public void Retract() {

    }

    public void handlePID() {

    }
}