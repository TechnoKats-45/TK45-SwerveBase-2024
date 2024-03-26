package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AmpShoot extends Command 
{
    private Shooter s_Shooter;
    private Joystick rumbleController;

    /** Creates a new TeleopFeeder. */
    public AmpShoot(Shooter s_Shooter, Joystick rumbleController) 
    {
        this.s_Shooter = s_Shooter;
        this.rumbleController = rumbleController;
        
        addRequirements(s_Shooter);

        s_Shooter.setTarget(0.15);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {        
        s_Shooter.holdAmpTarget();

        if(s_Shooter.upToSpeed())
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