package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command 
{    
    private Intake s_Intake;

    public TeleopIntake(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    @Override
    public void execute() 
    {
        s_Intake.runIntake(0);  // sets and holds target speed of 0
        //SmartDashboard.putBoolean("TEST INTAKE", false);
    }
}