package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
    private final Joystick rumbleController;
    private Timer timer;
    private final double interval = 0.005; // 5ms

    Limelight s_Limelight;

    /** Creates a new AutoIntake Command. */
    public AutoIntake(Intake s_Intake, Feeder s_Feeder, Shoulder s_Shoulder, Limelight s_Limelight, Joystick rumbleController) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Intake = s_Intake;
        this.s_Shoulder = s_Shoulder;
        this.s_Limelight = s_Limelight;
        this.rumbleController = rumbleController;
        timer = new Timer();

        addRequirements(s_Feeder, s_Intake);
        // Called when the command is initially scheduled.
        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
    }

    @Override
    public void initialize() 
    {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (timer.get() >= interval) // TODO - test if this actually works (Should allow intake speed up to 100%)
        {
            timer.reset();  // Reset the timer after checking the interval

            s_Shoulder.holdTarget();
            if (!s_Intake.detectGamePiece() && !s_Feeder.detectGamePiece()) 
            {  // Check if we have a game piece already
                s_Intake.runIntake(Constants.Intake.intakeSpeed);
            } 
            else 
            {
                s_Intake.runIntake(0);
                // We already have a game piece
                // TODO - add diagnostics
            }

            s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
        }
    }

    public boolean isFinished() 
    {
        return s_Intake.detectGamePiece();  // End when a GamePiece is detected in intake
    }

    @Override
    public void end(boolean interrupted) 
    {
        s_Shoulder.setTarget(Constants.Shoulder.handoffAngle);
        s_Shoulder.holdTarget();
        s_Limelight.setLEDMode(Constants.Limelight.LED_BLINK);
        rumbleController.setRumble(Joystick.RumbleType.kBothRumble, 1);
    }
}
