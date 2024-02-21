package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.Constants;

public class AutoFeed extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;

    /** Creates a new TeleopFeeder. */
    public AutoFeed(Intake s_Intake, Feeder s_Feeder, Shoulder s_Shoulder) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        
        addRequirements(s_Feeder, s_Intake, s_Shoulder);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Shoulder.holdTarget();  // Hold the shoulder at the target angle
        
        if(!s_Feeder.detectGamePiece() && s_Intake.detectGamePiece() && s_Shoulder.isAligned())   // If no GP in feeder, and yes GP in intake, and yes shoulder angle correct
        {
            s_Intake.runIntake(Constants.Intake.hanfoffSpeed);
            s_Feeder.runFeeder(Constants.Feeder.handoffSpeed);
        }
        else
        {
            // TODO - add diagnostics
        }
    }

    @Override
    public boolean isFinished()
    {
        return s_Feeder.detectGamePiece();  // End when GamePiece is detected in Feeder
    }
}