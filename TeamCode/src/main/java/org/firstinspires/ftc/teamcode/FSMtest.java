package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "FSMTest")
public class FSMtest extends LinearOpMode {
    enum State {
        FORWARD,
        TURN,
        IDLE
    }

    State curentState = State.IDLE;
    Pose2d startPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0, 24))
                .build();
        curentState = State.FORWARD;
        drive.followTrajectoryAsync(forward);
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            switch(curentState){
                case FORWARD:
                    if(!drive.isBusy()){
                        curentState = State.TURN;
                        drive.turnAsync(Math.toRadians(90));
                    }
                    break;
                case TURN:
                    if(!drive.isBusy()){
                        curentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
