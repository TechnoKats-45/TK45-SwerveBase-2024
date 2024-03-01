package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.Constants;

public class FeederDown extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;

    /** Creates a new TeleopFeeder. */
    public FeederDown(Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        
        addRequirements(s_Feeder);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {        
        s_Feeder.runFeeder(-Constants.Feeder.handoffSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return s_Feeder.detectGamePiece();  // End when GamePiece is detected in Feeder
    }

    @Override
    public void end(boolean interrupted)
    {
        SmartDashboard.putBoolean("Feed Down Interupted", interrupted);
    }
}