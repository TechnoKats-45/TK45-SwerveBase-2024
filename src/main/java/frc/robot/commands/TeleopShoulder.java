package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopShoulder extends Command 
{    
    private Shoulder s_Shoulder;    
    private Joystick operator;
    private Joystick driver;

    public TeleopShoulder(Shoulder s_Shoulder, Joystick operator, Joystick driver) 
    {
        this.s_Shoulder = s_Shoulder;
        this.operator = operator;
        this.driver = driver;
        addRequirements(s_Shoulder);
    }

    @Override
    public void execute() 
    {
        s_Shoulder.moveAngle(operator, driver);          // Move to appropriate angle   // TODO - write new moveAngle in shoulder subsystem
    }
}