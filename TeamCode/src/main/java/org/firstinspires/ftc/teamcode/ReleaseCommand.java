package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class ReleaseCommand extends CommandBase {
    private final Arm m_arm;
    public ReleaseCommand(Arm subsystem){

        m_arm = subsystem;
        addRequirements(m_arm);
    }
    @Override
    public void initialize(){
        m_arm.release();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
