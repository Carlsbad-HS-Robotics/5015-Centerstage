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
    //stages of our auto
    enum State{
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,
        TRAJECTORY_6,
        TRAJECTORY_7,
        TRAJECTORY_8,
        IDLE
    }
    int objectPosition;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        drive = new SampleMecanumDrive(hardwareMap);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //declaring our trajectories
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-35, -35))
                .build();
        TrajectorySequence traj1L = drive.trajectorySequenceBuilder(new Pose2d(-35, -35, Math.toRadians(90.00))) //
                .lineTo(new Vector2d(-35, -35))
                .build();
        TrajectorySequence traj1M = drive.trajectorySequenceBuilder(new Pose2d(-40.80, -34.74, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-41.67, -24.05))
                .build();
        TrajectorySequence traj1R = drive.trajectorySequenceBuilder(new Pose2d(-37.77, -33.15, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-24.34, -33.15))
                .build();






        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(-36.18, -35.89, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.32, -11.92), Math.toRadians(90.35))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(-35.60, -11.92, Math.toRadians(90.00)))
                .lineTo(new Vector2d(9.17, -11.63))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(-35.60, -11.92, Math.toRadians(90.00)))
                .lineTo(new Vector2d(9.17, -11.63))
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(-35.60, -11.92, Math.toRadians(90.00)))
                .lineTo(new Vector2d(9.17, -11.63))
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(-35.60, -11.92, Math.toRadians(90.00)))
                .lineTo(new Vector2d(9.17, -11.63))
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d(53.08, -35.46, Math.toRadians(90.00)))
                .splineTo(new Vector2d(14.37, -6.14), Math.toRadians(90.00))
                .splineTo(new Vector2d(-45.14, 5.27), Math.toRadians(169.15))
                .splineTo(new Vector2d(-66.08, 57.85), Math.toRadians(111.72))
                .build();





        //queue first trajectory
        objectPosition = 1;
        drive.followTrajectorySequenceAsync(traj1);
        State currentState = State.TRAJECTORY_1;
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            switch(currentState){
                case TRAJECTORY_1:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_2; // switches to next stage after the drivetrain stops moving
                        if(objectPosition == 1){
                            drive.followTrajectorySequenceAsync(traj1L);
                        } else if(){
                        if(objectPosition==1){
                            drive.followTrajectorySequenceAsync(traj1M);
                        } else if (){
                        if(objectPosition==1){
                            drive.followTrajectorySequenceAsync(traj1R);
                        } else if (){

                        }
                        }
                        }
                    }
                    break;
                case TRAJECTORY_2:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectorySequenceAsync(traj3);
                    }
                    break;
                case TRAJECTORY_3:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectorySequenceAsync(traj4);
                    }
                    break;
                case TRAJECTORY_4:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_5;
                        drive.followTrajectorySequenceAsync(traj5);
                    }
                    break;
                case TRAJECTORY_5:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectorySequenceAsync(traj6);
                    }
                    break;
                case TRAJECTORY_6:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_7;
                        drive.followTrajectorySequenceAsync(traj7);
                    }
                    break;
                case TRAJECTORY_7:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_8;
                        drive.followTrajectorySequenceAsync(traj8);
                    }
                    break;
                case TRAJECTORY_8:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;

                    }
                case IDLE:

                    break;
            }

            //Code goes here:



            //code at the end of the loop to
            drive.update();
        }
    }
   // void forward(5000)





}
