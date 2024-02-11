package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robo.Arm;

@TeleOp(name = "armtEsting")
public class MoveaServo extends LinearOpMode {
    Arm arm_subsystem;
    @Override
    public void runOpMode() throws InterruptedException {
    arm_subsystem = new Arm(hardwareMap);
    arm_subsystem.elbow0.setInverted(true);
    waitForStart();
    String direction = "";
    while(opModeIsActive() && !isStopRequested()){
        if(gamepad1.a){
            arm_subsystem.elbow0.rotateByAngle(2);
            direction = "CLOCKWISE";
        } else if (gamepad1.b)
        {
            arm_subsystem.elbow0.rotateByAngle(-2);
            direction = "COUNTERCLOCKWISE";
        } else {
            arm_subsystem.elbow0.rotateByAngle(0);
        }
        telemetry.addData("direction:", direction);
        telemetry.update();
        }
    }
}
