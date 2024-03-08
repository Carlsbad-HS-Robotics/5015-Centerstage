/*package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robo.Arm;

@TeleOp(name = "armtest")
public class armTest extends LinearOpMode {
int MIN_ANGLE = 0;
int MAX_ANGLE = 3000;
    Arm arm_subsystem;
    @Override
    public void runOpMode() throws InterruptedException {
    arm_subsystem = new Arm(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            arm_subsystem.setElbowPosition(arm_subsystem.getElbowPosition()+0.001*gamepad2.right_stick_y);
            arm_subsystem.update();
            telemetry.addData("elbow positon", arm_subsystem.getElbowPosition());
            telemetry.update();
        }
    }


}
*/