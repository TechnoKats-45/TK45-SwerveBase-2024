package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;

public class TeleopVision extends Command
{
    private Vision s_Vision;
    private Joystick operator;
    private Joystick driver;
    private Swerve s_Swerve;

    /** Creates a new TeleopVision. */
    public TeleopVision(Vision s_Vision, Joystick operator, Joystick driver) 
    {
        this.s_Vision = s_Vision;
        this.operator = operator;
        this.driver = driver;
        //addRequirements(s_Vision, s_Swerve);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Vision.searchForTarget();   // TODO - Update ???
    }
}
