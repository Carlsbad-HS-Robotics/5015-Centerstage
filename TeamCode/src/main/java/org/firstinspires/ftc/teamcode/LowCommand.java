package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LowCommand extends CommandBase {
    private final Arm m_arm;
    public LowCommand(Arm subsystem){

        m_arm = subsystem;
        addRequirements(m_arm);
    }
    @Override
    public void initialize(){
        m_arm.low();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
