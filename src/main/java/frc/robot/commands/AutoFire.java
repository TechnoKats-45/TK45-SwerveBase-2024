package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AutoFire extends Command 
{
    private Feeder s_Feeder;
    private Shooter s_Shooter;
    private boolean finished = false;

    public AutoFire(Feeder s_Feeder, Shooter s_Shooter) 
    {
        this.s_Feeder = s_Feeder;
        this.s_Shooter = s_Shooter;
        addRequirements(s_Feeder);  // Only require Feeder, only going to read from other subsystems
                                    // adding other subsystems as requirements will cause them to be disabled
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        //s_Feeder.runFeeder(Constants.Feeder.speakerFeedSpeed);

        if(RobotState.isAutonomous())   // If in auto, run for set time, then stop
        {
            s_Shooter.runShooter(Constants.Shooter.shooterSpeed);
            Timer.delay(0.5);
            s_Feeder.runFeeder(Constants.Feeder.speakerFeedSpeed);
            Timer.delay(0.25);
            s_Feeder.runFeeder(0);
            s_Shooter.runShooter(0);
            finished = true;
        }
        else if (RobotState.isTeleop())
        {
            s_Feeder.runFeeder(Constants.Feeder.speakerFeedSpeed);
        }
        else
        {
            // I shouldn't be here
        }
    }

    @Override
    public boolean isFinished()
    {
        if(RobotState.isAutonomous() && finished == true)
        {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Feeder.runFeeder(0);
        s_Shooter.coastToZero();
    }
}