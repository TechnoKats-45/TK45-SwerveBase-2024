package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AmpScore extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;
    private Shooter s_Shooter;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopFeeder. */
    public AmpScore(Feeder s_Feeder, Joystick operator, Joystick driver) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Shooter = s_Shooter;
        this.operator = operator;
        this.driver = driver;
        addRequirements(s_Feeder, s_Shoulder, s_Shooter);
        // Called when the command is initially scheduled.
    }

    @Override
    public void execute() 
    {
        // If note in feeder, allow, if not, prompt AutoIntake or AutoFeed // TODO - Update to actual command names
        if(s_Feeder.detectGamePiece())
        {
            // Allow Amp Score
            s_Shoulder.setAngle(Constants.ampScoreAngle); // Set shoulder to amp score angle    // TODO - Update to actual angle
            // Output game piece when prompted (button press)
        }
        else
        {
            // Do nothing
        }
    }

}