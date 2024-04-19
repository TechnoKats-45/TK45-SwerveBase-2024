package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class LoftedPass extends Command 
{
    private Shooter s_Shooter;
    private Joystick rumbleController;
    private Shoulder s_Shoulder;

    /** Creates a new TeleopFeeder. */
    public LoftedPass(Shooter s_Shooter, Joystick rumbleController, Shoulder s_Shoulder) 
    {
        this.s_Shooter = s_Shooter;
        this.rumbleController = rumbleController;
        this.s_Shoulder = s_Shoulder;
        
        addRequirements(s_Shooter, s_Shoulder);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Shooter.setTarget(0.45);
        s_Shooter.holdAmpTarget();

        s_Shoulder.setTarget(Constants.Shoulder.groundParallelAngle - Constants.Shoulder.loftedPassAngle);
        s_Shoulder.holdTarget();

        if(s_Shooter.upToSpeed() && s_Shoulder.isAligned())
        {
            // rumble
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 1);
        }
        else
        {
            // don't rumble
            rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Shooter.coastToZero();
        rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);
    }
}