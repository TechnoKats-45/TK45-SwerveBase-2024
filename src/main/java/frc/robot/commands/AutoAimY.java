package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;

public class AutoAimY extends Command 
{

    private final Shoulder s_Shoulder;
    private final Limelight s_limelight;

    public AutoAimY(Shoulder s_Shoulder, Limelight s_limelight) 
    {
        this.s_Shoulder = s_Shoulder;
        this.s_limelight = s_limelight;
        addRequirements(s_Shoulder, s_limelight);
    }

    @Override
    public void execute() 
    {
        //TODO: VERIFY
        s_Shoulder.setAlignedAngle(s_limelight.getRX(), s_limelight.getRZ(), s_limelight.tagExists());
    }
}