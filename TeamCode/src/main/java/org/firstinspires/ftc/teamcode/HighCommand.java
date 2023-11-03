package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class HighCommand extends CommandBase {
    private final Arm m_arm;
    public HighCommand(Arm subsystem){

        m_arm = subsystem;
        addRequirements(m_arm);
    }
    @Override
    public void initialize(){
        m_arm.high();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
