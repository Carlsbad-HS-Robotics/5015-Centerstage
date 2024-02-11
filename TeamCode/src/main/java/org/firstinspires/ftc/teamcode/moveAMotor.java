package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp  (name = "motorTest")
public class moveAMotor extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            motor.setPower(gamepad1.left_stick_y);
        }

    }
}
