package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.Constants;

public class ArmSetTilTarget extends Command 
{
    private Shoulder s_Shoulder;
    private double targetAngle;

    public ArmSetTilTarget(Shoulder s_Shoulder, double targetAngle) 
    {
        this.s_Shoulder = s_Shoulder;
        this.targetAngle = targetAngle;
        
        addRequirements(s_Shoulder);

        s_Shoulder.setTarget(targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {        
        s_Shoulder.setTarget(targetAngle);
        s_Shoulder.holdTarget();
    }

    @Override
    public boolean isFinished()
    {
        if(s_Shoulder.isAligned())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
    }
}