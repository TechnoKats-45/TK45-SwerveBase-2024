package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopArm extends Command 
{    
    private Arm s_Arm;    
    private Joystick operator;
    private Joystick driver;

    public TeleopArm(Arm s_Arm, Joystick operator, boolean aSup, boolean ySup, Joystick driver) 
    {
        this.s_Arm = s_Arm;
        this.operator = operator;
        this.driver = driver;
        addRequirements(s_Arm);
    }

    @Override
    public void execute() 
    {
        s_Arm.moveAngle(operator, driver);          // Move to appropriate angle
    }
}