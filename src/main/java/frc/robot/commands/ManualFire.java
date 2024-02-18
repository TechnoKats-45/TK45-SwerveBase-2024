package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;
import frc.robot.Constants;

public class ManualFire extends Command 
{
    private Feeder s_Feeder;
    private Shoulder s_Shoulder;

    /** Creates a new TeleopFeeder. */
    public ManualFire(Shoulder s_Shoulder, Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Shoulder = s_Shoulder;
        
        addRequirements(s_Feeder, s_Shoulder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(s_Shoulder.getAngle() >= 0)   // If in speaker mode -> Shooter pointing up
        {
            s_Feeder.setSpeed(Constants.Feeder.speakerFeedSpeed);
        }
        else if(s_Shoulder.getAngle() < 0)  // If in amp mode -> Feeder pointing up
        {
            s_Feeder.setSpeed(1);   // change to "Constants.Feeder.ampShootSpeed"
        }
        else
        {
            // ERROR
        }
    }

    @Override
    public boolean isFinished()
    {
        return !s_Feeder.detectGamePiece(); // End once GP is no longer detected
    }
}