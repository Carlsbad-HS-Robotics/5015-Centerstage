package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="😂😎")
public class Motorsada extends LinearOpMode {

    SampleMecanumDrive drive;
    AprilTagProcessor atag1;
    VisionPortal vPortal1;
    List<AprilTagDetection> atagDetections;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        atag1 = AprilTagProcessor.easyCreateWithDefaults();
        vPortal1 = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"Webcam 1"),atag1);
        vPortal1.setProcessorEnabled(atag1,true);

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){

            atagDetections = atag1.getDetections();
            for (AprilTagDetection detection : atagDetections) {
                if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    if(detection.id == 6){
                        }

                    }
                }
            telemetry.update();

            }
        }



    }

