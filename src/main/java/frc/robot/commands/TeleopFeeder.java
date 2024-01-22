package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class TeleopFeeder extends Command 
{
    private Feeder s_Feeder;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopFeeder. */
    public TeleopFeeder(Feeder s_Feeder, Joystick operator, Joystick driver) 
    {
        this.s_Feeder = s_Feeder;
        this.operator = operator;
        this.driver = driver;
        // addRequirements(s_Feeder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Feeder.runFeeder(operator, driver);   // TODO - Update to Feed Command
    }

}