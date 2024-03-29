package frc.robot.commands;

import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class AutoJiggler extends Command 
{
    private Feeder s_Feeder;


    /** Creates a new AutoIntake Command. */
    public AutoJiggler(Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        
        addRequirements(s_Feeder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Feeder.setTarget(0.25);
        s_Feeder.holdTarget();

        Timer.delay(0.25);

        s_Feeder.setTarget(-.25);
        s_Feeder.holdTarget();

        while(s_Feeder.detectGamePiece() == false)
        {
            s_Feeder.setTarget(0);
            s_Feeder.holdTarget();
        }
    }
}