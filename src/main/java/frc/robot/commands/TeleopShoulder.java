package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Shoulder;


public class TeleopShoulder extends Command 
{    
    private Shoulder s_Shoulder;

    public TeleopShoulder(Shoulder s_Shoulder) 
    {
        this.s_Shoulder = s_Shoulder;
        addRequirements(s_Shoulder);
    }

    @Override
    public void execute() 
    {
        s_Shoulder.holdTarget();
    }
}