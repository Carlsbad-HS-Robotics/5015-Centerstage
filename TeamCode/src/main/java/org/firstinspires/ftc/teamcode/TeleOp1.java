package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TeleOp")
public class TeleOp1 extends LinearOpMode {
    GamepadEx driver;
    GamepadEx coDriver;
    private Arm arm_subsystem;
    private GrabCommand m_lowCommand;
    private Button low_button, high_button;
    @Override
    public void runOpMode() throws InterruptedException {
    driver = new GamepadEx(gamepad1);
    coDriver = new GamepadEx(gamepad2);
   // Arm arm = new Arm(hardwareMap, "grabCommand");
    new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312).setInverted(true);
    new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
    new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312).setInverted(true);
    new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312).setInverted(true);
        /*MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312)
        );
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        TriggerReader rTrigger = new TriggerReader(
                coDriver, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        TriggerReader lTrigger = new TriggerReader(
                coDriver, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        */

        /*
        arm_subsystem = new Arm(hardwareMap, "armcontrol");
        m_lowCommand = new GrabCommand(arm_subsystem);
        low_button = (new GamepadButton(coDriver, GamepadKeys.Button.A))
                .whenPressed(m_lowCommand);
*/

        waitForStart();
        if(isStopRequested()) return;
        while(!isStopRequested()){
            drive.driveFieldCentric(
                    driver.getLeftX(),
                    driver.getLeftY(),
                    driver.getRightX(),
                    imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    false
            );
            telemetry.addData("left X",driver.getLeftX());
            telemetry.addData("left Y", driver.getLeftY());
            telemetry.update();
        }
    }
}
