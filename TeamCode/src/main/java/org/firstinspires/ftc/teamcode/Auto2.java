package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="automode1")
public class Auto2 extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        drive = new SampleMecanumDrive(hardwareMap);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(36)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .forward(48)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(3)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(36)
                .build();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            //Code goes here:
            drive.followTrajectory(traj1);
            drive.turn(90);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj4);
            //put the pixel
            drive.followTrajectory(traj3);
            drive.turn(90);
            //put the pixel
            break;
        }
    }
   // void forward(5000)

    void left(){
        frontRightMotor.setPower(1);
        frontLeftMotor.setPower(-1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(-1);

    }
    void right(){
        frontRightMotor.setPower(-1);
        frontLeftMotor.setPower(1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(1);

    }
    void forward(){
        frontRightMotor.setPower(1);
        frontLeftMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(1);
    }
    void backward(){
        frontRightMotor.setPower(-1);
        frontLeftMotor.setPower(-1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(-1);
    }



}
