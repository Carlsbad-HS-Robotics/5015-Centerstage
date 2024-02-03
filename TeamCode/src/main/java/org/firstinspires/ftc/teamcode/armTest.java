package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "armtest")
public class armTest extends LinearOpMode {
int MIN_ANGLE = 0;
int MAX_ANGLE = 3000;
    private ServoEx elbow0;
    private ServoEx elbow1;
    private ServoEx wrist;
    @Override
    public void runOpMode() throws InterruptedException {
        elbow0 =
                new SimpleServo(hardwareMap, "elbow0", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        elbow1 =
                new SimpleServo(hardwareMap, "elbow1", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        wrist =
                new SimpleServo(hardwareMap, "wrist", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        elbow1.setInverted(true);



        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            elbow0.setPosition(1);
            elbow1.setPosition(1);
            wrist.setPosition(1);
            telemetry.update();
        }
    }
}
