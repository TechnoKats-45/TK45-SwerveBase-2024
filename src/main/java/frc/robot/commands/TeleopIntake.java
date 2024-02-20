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

import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command 
{    
    private Intake s_Intake;

    public TeleopIntake(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    @Override
    public void execute() 
    {
        s_Intake.runIntake(0);  // sets and holds target speed of 0
        SmartDashboard.putBoolean("TEST INTAKE", false);
    }
}