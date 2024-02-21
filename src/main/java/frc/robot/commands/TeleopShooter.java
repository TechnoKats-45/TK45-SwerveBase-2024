package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Shooter;


public class TeleopShooter extends Command 
{    
    private Shooter s_Shooter;

    public TeleopShooter(Shooter s_Shooter) 
    {
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute() 
    {
        s_Shooter.runShooter(0);
    }
}