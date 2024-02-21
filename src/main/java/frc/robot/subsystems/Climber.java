package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;
    public double kP, kI, kD;
    private PIDController m_pidController = new PIDController(kP, kI, kD);
    private RelativeEncoder encoder;

    double target = 0;
    int angle;

    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setSmartCurrentLimit(40);
        climber.setInverted(false);
        encoder = climber.getEncoder();
    }

    public double getHeight()
    {
        return encoder.getPosition() * 360 * Constants.Climber.kInchesPerRotation;    // TODO - update InchesPerRotation
    }

    public void setTarget(double height)  // For external height setting
    {
        target = height;
    }

    public void holdTarget()    // For holding the climber at the target height
    {
        climber.set(m_pidController.calculate(getHeight(), target));
    }

    public void runClimber(double speed)
    {
        climber.set(speed);
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Climber Height", getHeight());
        SmartDashboard.putNumber("Climber Target", target);
    }
}
