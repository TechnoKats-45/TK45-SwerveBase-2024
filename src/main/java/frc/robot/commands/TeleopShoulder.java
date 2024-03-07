package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Shoulder;


public class TeleopShoulder extends Command 
{    
    private Shoulder s_Shoulder;

    public TeleopShoulder(Shoulder s_Shoulder) 
    {
        this.s_Shoulder = s_Shoulder;
        addRequirements(s_Shoulder);
    }

    @Override
    public void execute() 
    {
        s_Shoulder.holdTarget();
    }
}