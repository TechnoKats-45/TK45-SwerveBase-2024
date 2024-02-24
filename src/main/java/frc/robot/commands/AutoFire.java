package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class AutoFire extends Command 
{
    private Feeder s_Feeder;
    private Limelight s_Limelight;  // Probs can remove limelight from this whole command class
    private Shoulder s_Shoulder;

    public AutoFire(Feeder s_Feeder, Limelight s_Limelight, Shoulder s_Shoulder) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        addRequirements(s_Feeder);  // Only require Feeder, only going to read from other subsystems
                                    // adding other subsystems as requirements will cause them to be disabled
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        /*
        // Check if Shoulder is at the target angle
        // Check if swerve is at the target angle
        if(s_Shoulder.isAligned() && s_Limelight.isAlignedX())
        {
            s_Feeder.runFeeder(Constants.Feeder.speakerFeedSpeed);  // If both true, fire
        }
        else    // If not, do nothing
        {
            // TODO - add diagnostics
            // Maybe rumbles the controller   
        }
        */
        s_Feeder.runFeeder(Constants.Feeder.speakerFeedSpeed);

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}