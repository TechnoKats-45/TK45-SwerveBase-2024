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

public class AutoAim extends Command 
{
    private Shoulder s_Shoulder;
    private Joystick operator;
    private Joystick driver;

    /** Creates a new TeleopFeeder. */
    public AutoAim(Shoulder s_Shoulder, Joystick operator, Joystick driver) 
    {
        this.s_Shoulder = s_Shoulder;
        this.operator = operator;
        this.driver = driver;
        
        addRequirements(s_Shoulder);
        // Called when the command is initially scheduled.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        // TODO - Add auto aim code
        // If Vision Target to the left, turn left
        // If Vision Target to the right, turn right
        // If Vision Target in the center, stop

        // measure distnae to target
        // calculate vertical angle to target
        // move shoulder to angle
    }
}