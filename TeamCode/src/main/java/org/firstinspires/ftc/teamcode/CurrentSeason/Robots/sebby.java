package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.RoadRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.GamepadSettings;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveLibrary;
import org.opencv.core.Point;

public class Sebby extends AbstractRobot {

    public RoadRunnerMecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public CurveLibrary curveLibrary;

    public Sebby(OpMode opMode) {
        super(opMode);
        curveLibrary = new CurveLibrary();
        drive = new RoadRunnerMecanumDrive(this);

        //TODO : make this match with out outtake and intake
        outtake = new Outtake(this, "lsm", "rsm", "sb", "st", "sl", "sr");
        intake = new Intake(this, "iwl", "iwr");
    }
}