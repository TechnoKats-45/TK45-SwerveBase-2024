package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Feeder;


public class TeleopFeeder extends Command 
{    
    private Feeder s_Feeder;

    public TeleopFeeder(Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        addRequirements(s_Feeder);
    }

    @Override
    public void execute() 
    {
        s_Feeder.runFeeder(0);  // sets and holds target speed of 0
        //SmartDashboard.putBoolean("TEST FEEDER", false);
    }
}