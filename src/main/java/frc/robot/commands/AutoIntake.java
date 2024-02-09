package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AutoIntake extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;
    private Shooter s_Shooter;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopFeeder. */
    public AutoIntake(Intake s_Intake, Shoulder s_Shoulder, Feeder s_Feeder, Shooter s_Shooter, Joystick driver, Joystick operator) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Shooter = s_Shooter;
        this.operator = operator;
        this.driver = driver;
        
        addRequirements(s_Feeder, s_Intake, s_Shoulder, s_Shooter);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(!s_Intake.detectGamePiece() && !s_Feeder.detectGamePiece()) // If gamepiece is NOT in intake and NOT in feeder
        {
            s_Intake.intakeUntilSeen();
            s_Shoulder.setAngle(Constants.Shoulder.handoffAngle);
            s_Feeder.setSpeed(0);
        }
        else if(s_Intake.detectGamePiece() && s_Feeder.detectGamePiece() && s_Shoulder.getAngle() != Constants.Shoulder.handoffAngle)  // If gamepiece IS in intake, but NOT in feeder, and shoulder angle is NOT correct.
        {
            s_Shoulder.setAngle(Constants.Shoulder.handoffAngle);
            s_Intake.setSpeed(0);
        }
        else if(s_Intake.detectGamePiece() && s_Feeder.detectGamePiece() && s_Shoulder.getAngle() == Constants.Shoulder.handoffAngle)   // If gamepiece IS in intake, but NOT in feeder, and shoulder angle IS correct.
        {
            s_Intake.setSpeed(Constants.Intake.hanfoffSpeed);
            s_Feeder.feedUntilSeen();
        }
        else if(!s_Intake.detectGamePiece() && s_Feeder.detectGamePiece())   // If gamepiece is NOT in intake, but IS in feeder
        {
            s_Intake.setSpeed(0);
        }
        else    // ERROR
        {

        }
    }
}