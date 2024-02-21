package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Climber;


public class TeleopClimber extends Command 
{    
    private Climber s_Climber;

    public TeleopClimber(Climber s_Climber) 
    {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    @Override
    public void execute() 
    {
        s_Climber.runClimber(0);  // sets and holds target speed of 0
    }
}