// TODO add a header denoting who this code belongs to

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robo.Arm;

@TeleOp(name = "TeleOp")
public class TeleOp1 extends LinearOpMode {
    GamepadEx driver;
    GamepadEx coDriver;
    private Arm arm_subsystem;
    // TODO remove unused variables
    private Button low_button, high_button;
    private int slidePos = 0;
    Motor leftFront, rightFront, leftRear, rightRear;
    private MecanumDrive drive;
    private double armTimeDif = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO synchronize initialization code across all opmodes
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftRear = new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        rightRear = new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312);
        leftFront.setInverted(true);
        leftRear.setInverted(true);

        driver = new GamepadEx(gamepad1);
        coDriver = new GamepadEx(gamepad2);
        arm_subsystem = new Arm(hardwareMap);
        // TODO remove commented out code
/*
        ToggleButtonReader leftBumperToggle = new ToggleButtonReader(
                coDriver, GamepadKeys.Button.LEFT_BUMPER
        );
        ToggleButtonReader rightBumperToggle = new ToggleButtonReader(
                coDriver, GamepadKeys.Button.RIGHT_BUMPER
        );

 */
        /*
        Button claw = new GamepadButton(
                coDriver,GamepadKeys.Button.A
        );

         */
/*
        ToggleButtonReader armToggle = new ToggleButtonReader(
                coDriver, GamepadKeys.Button.B
        );

 */
        boolean leftChanged = false;
        boolean rightChanged = false;

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
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            // TODO refactor code for different robot functions, ie drive, arm, etc, into different java methods
            double y = -gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_y * 1.1;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            double rotX;
            double rotY;

            // Rotate the movement direction counter to the bot's rotation
            if (!gamepad1.left_bumper) {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            } else {
                rotX = x;
                rotY = y;
            }

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double multiplier = 1 - gamepad1.right_trigger * 0.75;
            leftFront.set(frontLeftPower * multiplier);
            leftRear.set(backLeftPower * multiplier);
            rightFront.set(frontRightPower * multiplier);
            rightRear.set(backRightPower * multiplier);
            ////////////////////////////////////////////////////////////////////////////////////
            armTimeDif = arm_subsystem.armClock.nanoLifespan();
            arm_subsystem.armClock.reset();
            arm_subsystem.setSlidePower(coDriver.getLeftX());


            //arm_subsystem.setElbowPosition(arm_subsystem.getElbowPosition() + coDriver.getRightY());


            if (coDriver.getButton(GamepadKeys.Button.A)) {
                arm_subsystem.setIntake(1);
            } else {
                arm_subsystem.setIntake(0);
            }
            if (coDriver.getButton(GamepadKeys.Button.LEFT_BUMPER) && !leftChanged) {
                if (!arm_subsystem.getLeftState()) arm_subsystem.grabLeft();
                else arm_subsystem.releaseLeft();
                leftChanged = true;
            } else if (!coDriver.getButton(GamepadKeys.Button.LEFT_BUMPER)) leftChanged = false;

            if (coDriver.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !rightChanged) {
                if (!arm_subsystem.getRightState()) arm_subsystem.grabRight();
                else arm_subsystem.releaseRight();
                rightChanged = true;
            } else if (!coDriver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) rightChanged = false;
            /*
            else if(coDriver.getButton(GamepadKeys.Button.A)){
                arm_subsystem.low();
            } else if (coDriver.getButton(GamepadKeys.Button.B)){
                arm_subsystem.high();
            } else if(coDriver.getButton(GamepadKeys.Button.Y)){
                arm_subsystem.drop();

             */
            if (coDriver.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                arm_subsystem.hangDown();
            } else if (coDriver.getButton(GamepadKeys.Button.DPAD_UP)) {
                arm_subsystem.hangUp();

            } else if (coDriver.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                arm_subsystem.hangServoDown();
            } else if (coDriver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                arm_subsystem.hangServoUp();
            } else {
                //arm_subsystem.hangOff();
            }
            if (gamepad1.options) {
                imu.resetYaw();
            }
            /*
            leftBumperToggle.readValue();
            rightBumperToggle.readValue();

             */
            /*arm_subsystem.update();
//////////////////////////////////////////////////////////////////////////////////////////////
            telemetry.addData("left X", driver.getLeftX());
            telemetry.addData("left Y", driver.getLeftY());
            //telemetry.addData("leftB state:", leftBumperToggle.getState());
            //telemetry.addData("rightB state:", rightBumperToggle.getState());
            telemetry.addData("a:", coDriver.getButton(GamepadKeys.Button.A));
            telemetry.addData("b:", coDriver.getButton(GamepadKeys.Button.B));
            telemetry.addData("elbow anlge", arm_subsystem.getElbowAngle());
            telemetry.addData("elbow position", arm_subsystem.getElbowPosition());
            telemetry.addData("wrist angle,", arm_subsystem.getWristAngle());
            telemetry.addData("elbow Position", arm_subsystem.getElbowPosition());
            telemetry.update();

             */




        }
    }
}
