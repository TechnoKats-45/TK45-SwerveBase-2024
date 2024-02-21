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

import frc.robot.subsystems.Feeder;


public class TeleopFeeder extends Command 
{    
    private Feeder s_Feeder;

    public TeleopFeeder(Feeder s_Feeder) 
    {
        this.s_Feeder = s_Feeder;
        addRequirements(s_Feeder);
    }

    @Override
    public void execute() 
    {
        s_Feeder.runFeeder(0);  // sets and holds target speed of 0
        //SmartDashboard.putBoolean("TEST FEEDER", false);
    }
}