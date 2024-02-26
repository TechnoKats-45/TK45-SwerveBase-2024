package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class AutoIntake extends Command 
{
    private Feeder s_Feeder;
    private Intake s_Intake;
    private Shoulder s_Shoulder;
    Limelight s_Limelight;

    /** Creates a new AutoIntake Command. */
    public AutoIntake(Intake s_Intake, Feeder s_Feeder, Shoulder s_Shoulder, Limelight s_Limelight) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Limelight = s_Limelight;
        
        addRequirements(s_Feeder, s_Intake, s_Shoulder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);

        if(!s_Intake.detectGamePiece() && !s_Feeder.detectGamePiece())  // Check to see if we have a gamepiece already
        {
            s_Intake.runIntake(Constants.Intake.intakeSpeed);
        }
        else
        {
            s_Intake.runIntake(0);
            // We already have a gamepiece
            // TODO - add diagnostics
        }

        s_Shoulder.holdTarget();
    }

    public boolean isFinished()
    {
        return s_Intake.detectGamePiece();  // End when GamePiece is detected in intake
    }
}