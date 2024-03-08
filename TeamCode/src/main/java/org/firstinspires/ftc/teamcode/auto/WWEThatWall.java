package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="WWE that wall")
public class WWEThatWall extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        if(isStopRequested()) return;
        while(opModeIsActive()) {
            vroom();
            sleep(2700);
            stopMoving();
        }
    }
    public void vroom() {
        drive.leftFront .setPower(-0.5);
        drive.rightFront.setPower(-0.5);
        drive.leftRear  .setPower(-0.5);
        drive.rightRear .setPower(0.5);
    }
    public void stopMoving() {
        drive.leftFront .setPower(0);
        drive.leftRear  .setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear .setPower(0);
    }

}
