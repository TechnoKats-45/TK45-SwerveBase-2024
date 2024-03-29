package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;

public class AutoShoulder extends Command 
{
    private Limelight s_Limelight;  // Probs can remove limelight from this whole command class
    private Shoulder s_Shoulder;
    private double aprilTagHeightOffset;

    public AutoShoulder(Limelight s_Limelight, Shoulder s_Shoulder, double aprilTagHeightOffset) 
    {
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        this.aprilTagHeightOffset = aprilTagHeightOffset;
        addRequirements(s_Shoulder);    // adding other subsystems as requirements will cause them to be disabled
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(s_Limelight.tagExists())
        {
            s_Shoulder.setAlignedAngle(s_Limelight.getRZ(), s_Limelight.getRY() + aprilTagHeightOffset, s_Limelight.tagExists());   // s_Limelight.getRY() 
            s_Shoulder.holdTarget();
        }
        else
        {
            s_Shoulder.holdTarget();
        }
    }
}