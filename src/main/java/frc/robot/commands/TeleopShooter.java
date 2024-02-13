package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends Command 
{
    private Shooter s_Shooter;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopShooter. */
    public TeleopShooter(Shooter s_Shooter, Joystick operator, Joystick driver) 
    {
        this.s_Shooter = s_Shooter;
        this.operator = operator;
        this.driver = driver;
        addRequirements(s_Shooter);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Shooter.runShooter(operator, driver);
    }

}