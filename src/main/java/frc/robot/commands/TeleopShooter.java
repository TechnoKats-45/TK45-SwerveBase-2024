package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Shooter;


public class TeleopShooter extends Command 
{    
    private Shooter s_Shooter;

    public TeleopShooter(Shooter s_Shooter) 
    {
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute() 
    {
        s_Shooter.setTarget(0);
        s_Shooter.coastToZero();
    }
}