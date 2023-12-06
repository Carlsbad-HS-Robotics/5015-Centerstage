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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp")
public class TeleOp1 extends LinearOpMode {
    GamepadEx driver;
    GamepadEx coDriver;
    private Arm arm_subsystem;
    private Button low_button, high_button;
    Motor leftFront, rightFront, leftRear, rightRear;
    private MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftRear = new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        rightRear = new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312);
        rightFront.setInverted(true);
        leftRear.setInverted(true);

        driver = new GamepadEx(gamepad1);
        coDriver = new GamepadEx(gamepad2);
        arm_subsystem = new Arm(hardwareMap);

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
// Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        drive = new MecanumDrive(leftFront,rightFront,leftRear,rightRear);
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            drive.driveRobotCentric(
                    driver.getLeftX(),
                    driver.getLeftY(),
                    driver.getRightX()
            );

            if (coDriver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm_subsystem.grab();
            } else if (coDriver.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                arm_subsystem.release();
            }
            else if(coDriver.getButton(GamepadKeys.Button.A)){
                arm_subsystem.high();
            } else if (coDriver.getButton(GamepadKeys.Button.B)){
                arm_subsystem.low();
            } else if(coDriver.getButton(GamepadKeys.Button.Y)){
                arm_subsystem.drop();
            } else if(coDriver.getButton(GamepadKeys.Button.DPAD_DOWN)){
                arm_subsystem.hangDown();
            }else if (coDriver.getButton(GamepadKeys.Button.DPAD_UP)){
                arm_subsystem.hangUp();

            } else if(coDriver.getButton(GamepadKeys.Button.DPAD_LEFT)){
                arm_subsystem.hangServoDown();
            } else if(coDriver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                arm_subsystem.hangServoUp();
            }
            else{
                arm_subsystem.hangOff();
            }
            if (gamepad1.options) {
                imu.resetYaw();
            }

                telemetry.addData("left X", driver.getLeftX());
                telemetry.addData("left Y", driver.getLeftY());
                telemetry.addData("a state:", clawToggle.getState());
                telemetry.addData("b state:", armToggle.getState());
                telemetry.addData("a:", coDriver.getButton(GamepadKeys.Button.A));
                telemetry.addData("b:", coDriver.getButton(GamepadKeys.Button.B));
                telemetry.addData("elbow anlge", arm_subsystem.getElbowAngle());
                telemetry.addData("wrist angle,", arm_subsystem.getWristAngle());
                telemetry.update();
                armToggle.readValue();
                clawToggle.readValue();


            }
        }
    }

