package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;

    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setSmartCurrentLimit(40);
        climber.setIdleMode(IdleMode.kBrake);
        climber.setInverted(false);
    }

    public void runClimber(CommandXboxController controller)
    {
        if(controller.getRightY() < -Constants.STICK_DEADBAND)  // Up
        {
            climber.set(Constants.Climber.climberSpeed);  // Go up
        }
        else if(controller.getRightY() > Constants.STICK_DEADBAND) // Up
        {
            climber.set(-Constants.Climber.climberSpeed); // Go down
        }
        else
        {
            // Hold angle
            climber.set(0);
        }
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Climber Current", climber.getOutputCurrent());
    }
}
