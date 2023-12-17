package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * This is NOT an opmode.
 * <p>
 * This class is used to define all the specific hardware for a single robot.
 * In this case that robot is a Mecanum driven Robot.
 * Mecanum robots utilize 4 motor driven wheels.
 * <p>
 * <p>
 * <p>
 * Software 1 min and 45 second Elevator Speech:
 * - Github:
 * - We utilize an open source coding platform known as github. Github adds value to our team in 3 ways:
 * - Version Control:
 * - Allows us to keep track of multiple files, backup our code, and share it with the world
 * - Colaboration:
 * - Allow us to make our code open source to the interwebs.
 * - Backup:
 * - We will never lose a file again
 * - Computer vision
 * - Vuforia:
 * - Definition File
 * - Auto
 * - Teleop
 * - Past failures
 * - Tensorflow
 */
public class Definitions {
    public static final String VUFORIA_KEY = "AQFyZOr/////AAABmRL+3QMh6kiBj2OqwKGebApeLvS635fqPcuCrcT8QdD0u4Y4EtbuBcQx1GtwkPSykB4xBRu+ZM5NapeDwTwkVQdSVNjorl1ebJalwSv0gJcSUVDZy/S45PyYVMuwvl6hdI1sFTKwnejvz8eyyxEpaAv4FZCP99BakBW7reGXUYYIHyXgsBDFOpprXd8Ka0GmgHYixugvl9WkV3DK0f4H3TG0d93QR6uY9Yp7Iyr01XVJ+Oym7YRKEzEywe3O9HzOIA5j/fh3zTg9GSpWpdXq400rxgviEpncr3YnynCzm9PjwPy9K8rfROJz5/2ZcO8uWcjxCCdPLbreVKIeKrpqBtsGtNEr8X2dFNhwLYfq8cKr";
    public Motor leftFront = null;
    public Motor rightFront = null;
    public Motor leftRear = null;
    public Motor rightRear = null;
    public final double ticks_in_degree = 1421.1 / 360.0 * 1.3;
    //port 2 on the thing (ic2)
    RevIMU imu;

    double speedMultiplier = 1;







    final float mmPerInch = 25.4f;
    // Class Members
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */


    final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Definitions(HardwareMap Map) {
        //telemetry.addData("Status :", "hardware map");
        leftFront = new Motor(Map, "leftFront");
        rightFront = new Motor(Map, "rightFront");
        leftRear = new Motor(Map, "leftRear");
        rightRear = new Motor(Map, "rightRear");
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        imu = new RevIMU(Map);
        imu.init();

    }
    }




