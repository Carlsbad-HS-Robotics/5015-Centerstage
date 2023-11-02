package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="automode1")
public class Auto extends LinearOpMode {
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
        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                        .forward(20)
                                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            //Code goes here:
            drive.followTrajectory(left );
            sleep(2000);


            break;
        }
    }
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
