package org.firstinspires.ftc.teamcode;

import android.widget.ToggleButton;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
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
    private Button low_button, high_button;
    Motor leftFront, rightFront, leftRear, rightRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftRear = new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        rightRear = new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        rightRear.setInverted(true);
        leftFront.setInverted(true);
        leftRear.setInverted(true);

        driver = new GamepadEx(gamepad1);
        coDriver = new GamepadEx(gamepad2);
        arm_subsystem = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                leftFront,
                rightFront,
                leftRear,
                rightRear
        );
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        ToggleButtonReader clawToggle = new ToggleButtonReader(
                coDriver, GamepadKeys.Button.A
        );
        /*
        Button claw = new GamepadButton(
                coDriver,GamepadKeys.Button.A
        );

         */

        ToggleButtonReader armToggle = new ToggleButtonReader(
                coDriver, GamepadKeys.Button.B
        );
        /*
        Button arm = new GamepadButton(
                coDriver,GamepadKeys.Button.B
        );

         */
        /*;
        TriggerReader rTrigger = new TriggerReader(
                coDriver, GamepadKeys.Trigger.RIGHT

                eo[]\\_TRIGGER
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
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            drive.driveFieldCentric(
                    driver.getLeftX(),
                    driver.getLeftY(),
                    driver.getRightX(),
                    imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    false
            );
            if (clawToggle.getState()) {
                arm_subsystem.grab();
            } else {
                arm_subsystem.release();
            }
            if(armToggle.getState()){
                arm_subsystem.high();
            } else{
                arm_subsystem.low();
            }

                telemetry.addData("left X", driver.getLeftX());
                telemetry.addData("left Y", driver.getLeftY());
                telemetry.addData("a state:", clawToggle.getState());
                telemetry.addData("b state:", armToggle.getState());
                telemetry.addData("a:", coDriver.getButton(GamepadKeys.Button.A));
                telemetry.addData("b:", coDriver.getButton(GamepadKeys.Button.B));
                telemetry.update();
                armToggle.readValue();
                clawToggle.readValue();


            }
        }
    }

