package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;


public abstract class AbstractSubsystem {

    public AbstractRobot robot;

    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    public boolean usesConfig = false;

    //public SubsystemTeleOpConfig subsystemTeleOpConfigs;
    //public SubsystemAutoConfig subsystemAutoConfigs;

    public AbstractSubsystem(AbstractRobot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        this.telemetry = robot.telemetry;
    }

    public abstract void init() throws IOException;

    public abstract void start();

    public abstract void driverLoop();

    public abstract void stop();

    /*public SubsystemTeleOpConfig buildTeleOpConfig() {
        return new SubsystemTeleOpConfig(new ConfigItem[0]);
    }

    public SubsystemAutoConfig buildAutoConfig() {
        return new SubsystemAutoConfig(new ConfigItem[0]);
    }

    protected void buildConfigs() {
        subsystemTeleOpConfigs = buildTeleOpConfig();
        subsystemAutoConfigs = buildAutoConfig();
    }*/

}