package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    enum State{
        TRAJECTORY_1,
        IDLE
    }
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        drive = new SampleMecanumDrive(hardwareMap);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-36.19, -61.58, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-61.97, -61.59))
                .build();
        drive.followTrajectorySequenceAsync(traj1);
        State currentState = State.IDLE;
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            switch(currentState){
                case TRAJECTORY_1:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }

            //Code goes here:




            drive.update();
        }
    }
   // void forward(5000)





}
