package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;

public abstract class AbstractRobot {

    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;

    public ArrayList<AbstractSubsystem> subsystems;

    //public RobotTeleOpConfig robotTeleOpConfig;
    //public RobotAutoConfig robotAutoConfig;

    public Gamepad gamepad1, gamepad2;

    //public BHI260IMU imu;

    public AbstractRobot(OpMode opMode) {
        this.opMode =  opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        subsystems = new ArrayList<>();

    }

    public void init() throws IllegalAccessException, IOException {

        RobotLog.ii("before init robot 1", "woo");

        Field[] fields = this.getClass().getDeclaredFields();

        //ArrayList<SubsystemTeleOpConfig> subsystemTeleOpConfigs = new ArrayList<>();
        //ArrayList<SubsystemAutoConfig> subsystemAutoConfigs = new ArrayList<>();

        RobotLog.ii("before init field robot", "woo");

        for (Field f : fields) {
            RobotLog.ii("in init field robot", "woo");
            if (AbstractSubsystem.class.isAssignableFrom(f.getType())) {
                Object obj;
                RobotLog.ii("in if init robot", "woo");
                try {
                    RobotLog.ii("try init robot", "woo");
                    obj = f.get(this);
                }
                catch (IllegalAccessException e) {
                    RobotLog.ii("catch init robot", "woo");
                    throw new IllegalAccessException("make subsystems public");
                }

                if (obj != null) {
                    RobotLog.ii("if obj init robot", "woo");
                    subsystems.add((AbstractSubsystem)obj);
                    //TODO: load
                    /* if (((AbstractSubsystem)obj).usesConfig) {
                        ((AbstractSubsystem)obj).buildConfigs();
                        subsystemTeleOpConfigs.add(((AbstractSubsystem)obj).subsystemTeleOpConfigs);
                        subsystemAutoConfigs.add(((AbstractSubsystem)obj).subsystemAutoConfigs);
                    }*/
                }
            }
        }

        //robotTeleOpConfig = new RobotTeleOpConfig((SubsystemTeleOpConfig[])subsystemTeleOpConfigs.toArray());
        //robotAutoConfig = new RobotAutoConfig((SubsystemAutoConfig[])subsystemAutoConfigs.toArray());

        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        /*BHI260IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
        RobotLog.ii("before for init system robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            RobotLog.ii("in for init system robot", "woo");
            system.init();
        }
    }

    public final void start() {
        RobotLog.ii("before start robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            system.start();
            if (system.usesConfig) {
                //TODO: save
            }
        }


    }

    public final void driverLoop() {
        RobotLog.ii("before driverloop robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            telemetry.addData("", "-------" + system.getClass().getSimpleName() + "------- :");
            system.driverLoop();
        }
        telemetry.update();
    }

    public final void stop() {
        RobotLog.ii("before stop robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            system.stop();
        }
    }


}