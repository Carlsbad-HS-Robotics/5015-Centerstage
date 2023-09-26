package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Claw {
    public CRServo claw = null;
    boolean gripState;
    public Claw(HardwareMap Map) {
        claw = Map.crservo.get("claw");

    }

    public void Grip() {
        if (gamepad1.right_trigger > .5 && gripState != true){
            claw.setPower(1);
            gripState = true;
        }
        else{
            claw.setPower(0);
        }
    }
}
