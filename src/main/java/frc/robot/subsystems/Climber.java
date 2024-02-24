package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;
    public double kP, kI, kD;
    private PIDController m_pidController = new PIDController(kP, kI, kD);

    double target = 0;
    int angle;

    private final RelativeEncoder climberEncoder;


    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setSmartCurrentLimit(40);
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
            // TODO - add debug
        }
        target = angle;
    }

    public void holdTargetAngle()    // For holding the climber at the target height
    {
        climber.set(m_pidController.calculate(getAngle(), target));
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Climber Angle", getAngle());
        SmartDashboard.putNumber("Climber Target Angle", target);
    }
}
