package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.path.Line;

@TeleOp(name="toggletest")
public class ToggleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driver = new GamepadEx(gamepad1);
        ToggleButtonReader aTest = new ToggleButtonReader(
                driver, GamepadKeys.Button.A
        );
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            aTest.readValue();
            telemetry.addData("toggle:", aTest.getState());
            telemetry.update();

        }
    }
}
