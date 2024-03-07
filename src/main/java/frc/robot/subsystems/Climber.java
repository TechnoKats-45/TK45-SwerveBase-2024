package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;
    public double kP = 1, kI = 0, kD = 0;
    private PIDController m_pidController = new PIDController(kP, kI, kD);

    double target = 0;
    int angle;

    private final RelativeEncoder climberEncoder;


    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setSmartCurrentLimit(40);
        climber.setIdleMode(IdleMode.kBrake);
        climber.setInverted(false);
        climberEncoder = climber.getEncoder();
    }

    public double getAngle()
    {
        return climberEncoder.getPosition();
    }

    public void setTargetAngle(double angle)  // For external angle setting
    {
        if(angle > Constants.Climber.climberMaxAngle)
        {
            target = Constants.Climber.climberMaxAngle;
        }
        else if(angle < Constants.Climber.climberMinAngle)
        {
            target = Constants.Climber.climberMinAngle;
        }
        else
        {
            target = angle;
        }
    }

    public void holdTargetAngle()    // For holding the climber at the target height
    {
        climber.set(m_pidController.calculate(getAngle(), target)); // THIS IS BROKEN
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
        SmartDashboard.putNumber("Climber Angle", getAngle());
        SmartDashboard.putNumber("Climber Current", climber.getOutputCurrent());
    }
}
