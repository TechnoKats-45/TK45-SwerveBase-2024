package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class AutoJiggle extends Command 
{
    private Feeder s_Feeder;
    private double startTime;
    private boolean delayStarted = false;


    /** Creates a new AutoIntake Command. */
    public AutoJiggle(Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        
        addRequirements(s_Feeder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (!delayStarted) 
        {
            // Start the delay
            startTime = Timer.getFPGATimestamp();
            delayStarted = true;
        }

        if (Timer.getFPGATimestamp() - startTime >= Constants.Feeder.JiggleDelay) { // 25 milliseconds have passed
            // Delay is over, do what needs to be done after the delay
            System.out.println("25 milliseconds delay is over");
            delayStarted = false; // Reset the flag if you need to start the delay again later
        }

        // Drive down 1"
        // Drive down until sensor hit
        // Repeat
    }
}