package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class AutoShoulder extends Command 
{
    private Limelight s_Limelight;  // Probs can remove limelight from this whole command class
    private Shoulder s_Shoulder;

    public AutoShoulder(Limelight s_Limelight, Shoulder s_Shoulder) 
    {
        this.s_Limelight = s_Limelight;
        this.s_Shoulder = s_Shoulder;
        addRequirements(s_Shoulder);    // adding other subsystems as requirements will cause them to be disabled
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Shoulder.setAlignedAngle(s_Limelight.getRZ(), s_Limelight.getRY(), s_Limelight.tagExists());    // TODO - test this   // TODO - uncomment this
        s_Shoulder.holdTarget();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}