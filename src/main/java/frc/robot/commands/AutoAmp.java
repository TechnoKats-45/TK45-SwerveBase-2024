package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.Constants;

public class AutoAmp extends Command 
{
    private Feeder s_Feeder;
    private Shoulder s_Shoulder;

    /** Creates a new TeleopFeeder. */
    public AutoAmp(Feeder s_Feeder, Shoulder s_Shoulder) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Shoulder = s_Shoulder;
        
        addRequirements(s_Feeder, s_Shoulder);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {        
        // TODO - add check if we have a game piece???
        if(Math.abs(s_Shoulder.getAngle() - Constants.Shoulder.ampScoreAngle) < 1)   // If shoulder angle is within 1 degree of target angle
        {
            s_Feeder.runFeeder(Constants.Feeder.ampScoreSpeed); // Run feeder
        }
        else    // If shoulder is not at the correct angle
        {
            s_Shoulder.setTarget(Constants.Shoulder.ampScoreAngle);  // Set shoulder to correct angle
        }
        s_Shoulder.holdTarget();    // Hold / move shoulder at correct angle
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}