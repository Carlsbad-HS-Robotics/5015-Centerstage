package org.firstinspires.ftc.teamcode;

//cv imports
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.RedDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboctopi.cuttlefish.queue.*;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefish.localizer.*;
import com.roboctopi.cuttlefish.utils.*;
import com.roboctopi.cuttlefish.controller.*;
import com.roboctopi.cuttlefish.components.*;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import java.util.List;


/*TODO: add arm_subsystem into a conditional list
 *      update encoder ticks for sebby
 */

@Autonomous(name = "Red Backstage")
public class autoRedBackstage extends LinearOpMode {
    Arm arm_subsystem = new Arm(hardwareMap);
    CuttleRevHub controlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);
    CuttleRevHub expansionHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.EXPANSION_HUB);
    CuttleEncoder LFEncoder ;
    CuttleEncoder RFEncoder ;
    CuttleEncoder LBEncoder;
    CuttleEncoder RBEncoder;

    public CuttleMotor leftFrontMotor ;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor ;
    public CuttleMotor leftBackMotor  ;
    TaskList autoList = new TaskList();
    TaskQueue queue = new TaskQueue();
    MecanumController chassis;
    PTPController ptpController;
    OpenCvCamera webcam;
    RedDetection.ObjectPosition position = RedDetection.ObjectPosition.LEFT;
    List<AprilTagDetection> myAprilTagDetections;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int DESIRED_TAG_ID = 1;
    private int Encoder_Ticks = 560;

    public final static double mm_per_inch = 25.4;

    public void onInit()
    {

        //Drive Train
        leftFrontMotor  = controlHub.getMotor(0);
        rightFrontMotor = controlHub.getMotor(1);
        rightBackMotor  = controlHub.getMotor(2);
        leftBackMotor   = controlHub.getMotor(3);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        chassis = new MecanumController(
            rightFrontMotor,
            rightBackMotor ,
            leftFrontMotor ,
            leftBackMotor
        );

        //Encoder
        LFEncoder = controlHub.getEncoder(1, Encoder_Ticks);
        LBEncoder = controlHub.getEncoder(2, Encoder_Ticks);
        RFEncoder = controlHub.getEncoder(3, Encoder_Ticks);
        RBEncoder = controlHub.getEncoder(4, Encoder_Ticks);

        LBEncoder.setDirection(Direction.REVERSE);
        LFEncoder.setDirection(Direction.REVERSE);


        //PID
        /*ptpController = new PTPController(chassis, encoderLocalizer);

        ptpController.setTranslational_PD_ctrlr(new PID(
            translation_p_gain,
            0,
            translation_d_gain,
            0.0,
            maximumIntegralPower));
        ptpController.setRotational_PID_ctrlr(new PID(
            rotational_p_gain,
            rotational_i_gain,
            rotational_d_gain,
            0.0,
            maximumIntegralPower));*/

        ptpController.setTranslational_PD_ctrlr(new PID(
            0.02,  // Proportional
            0.0,   // Integral
            0.002, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));
        ptpController.setRotational_PID_ctrlr(new PID(
            3,  // Proportional
            0.0,   // Integral
            0.2, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));

        //Anti-Stall (continues to next WP if detects stalling)
        ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.2); // Maxmimum translational power where the bot is still stalled
        ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.2); // Maxmimum rotation power where the bot is still stalled
        ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(0.015);  // Maximum speed in m/s for the bot to be considered stalled
        ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.3); // Maximum rotation speed in rad/s for the bot to be considered stalled
    }
    public void runOpMode() {
        queue.addTask(new CustomTask(() -> {
            switch (position) {
                case LEFT:
                    arm_subsystem.grab();
                    autoList.addTask(new DelayTask(500));

                    autoList.addTask(new PointTask(
                            new Waypoint(
                                    new Pose(32 * mm_per_inch, 0, Math.PI / 2),
                                    0.5
                            ),
                            ptpController
                    ));

                    arm_subsystem.low();
                    arm_subsystem.update();

                    autoList.addTask(new DelayTask(1000));

                    arm_subsystem.release();
                    arm_subsystem.update();
                    break;
                case RIGHT:
                    arm_subsystem.grab();
                    queue.addTask(new DelayTask(500));

                    autoList.addTask(new PointTask(
                            new Waypoint(
                                    new Pose(32 * mm_per_inch, 0, Math.PI / 2),
                                    .5
                            ),
                            ptpController
                    ));
                    break;
            }
            return null;
        }));
    }
}