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

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;
    public double kP, kI, kD;
    private DutyCycleEncoder m_absoluteEncoder;
    private PIDController m_pidController = new PIDController(kP, kI, kD);

    double target = 0;
    int angle;

    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setSmartCurrentLimit(40);
        climber.setInverted(false);

        m_absoluteEncoder = new DutyCycleEncoder(Constants.Climber.ClimberEncoderPort);
    }

    public double getHeight()
    {
        return m_absoluteEncoder.getAbsolutePosition() * 360* Constants.Climber.kInchesPerRotation;    // TODO - update InchesPerRotation
    }

    public void setTarget(double height)  // For external height setting
    {
        target = height;
    }

    public void holdTarget()    // For holding the climber at the target height
    {
        climber.set(m_pidController.calculate(getHeight(), target));
    }

    public void diagnostics()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.add("Climber Height", getHeight());
        tab.add("Climber Target", target);
    }
}
