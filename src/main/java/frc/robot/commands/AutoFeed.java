package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class AutoFeed extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;
    private Limelight s_Limelight;
    private Joystick rumbleController;

    /** Creates a new TeleopFeeder. */
    public AutoFeed(Intake s_Intake, Feeder s_Feeder, Shoulder s_Shoulder, Limelight s_Limelight, Joystick rumbleController) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Limelight = s_Limelight;
        this.rumbleController = rumbleController;
        
        addRequirements(s_Feeder, s_Intake, s_Shoulder, s_Limelight);

        rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 1);
        s_Limelight.setLEDMode(Constants.Limelight.LED_BLINK);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {        
        s_Shoulder.holdTarget();
        
        if(!s_Feeder.detectGamePiece() && s_Shoulder.isAligned())   // If no GP in feeder, and yes GP in intake, and yes shoulder angle correct
        {
            s_Intake.runIntake(Constants.Intake.handoffSpeed);
            s_Feeder.runFeeder(Constants.Feeder.handoffSpeed);
        }
        else
        {
            // TODO - add diagnostics
        }

        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
    }

    @Override
    public boolean isFinished()
    {
        return s_Feeder.detectGamePiece();  // End when GamePiece is detected in Feeder
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Limelight.setLEDMode(Constants.Limelight.LED_OFF);
        rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 0);

        s_Feeder.runFeeder(0);
        s_Intake.runIntake(0);
        s_Shoulder.setTarget(s_Shoulder.getAngle());
        s_Shoulder.holdTarget();
    }
}