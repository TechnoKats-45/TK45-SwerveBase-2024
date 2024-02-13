package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class Climber extends SubsystemBase 
{
    private CANSparkMax climber;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    double target = 0;
    int angle;

    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

    private SparkPIDController m_pidController;
    private static final int kCPR = 8192;   // Counts per revolution for REV Through Bore Encoder

    private RelativeEncoder m_alternateEncoder;

    public Climber()
    {
        climber = new CANSparkMax(Constants.Climber.ClimberID, MotorType.kBrushless);

        climber.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = climber.getPIDController();

        /**
         * By default, the PID controller will use the Hall sensor from a NEO for its
         * feedback device. Instead, we can set the feedback device to the alternate
         * encoder object
         */
        m_pidController.setFeedbackDevice(m_alternateEncoder);

        // set PID coefficients
        m_pidController.setP(Constants.Climber.kP);
        m_pidController.setI(Constants.Climber.kI);
        m_pidController.setD(Constants.Climber.kD);
        m_pidController.setIZone(Constants.Climber.kIz);
        m_pidController.setFF(Constants.Climber.kFF);
        m_pidController.setOutputRange(Constants.Climber.kMinOutput, Constants.Climber.kMaxOutput);
    }

    public double getHeight()
    {
        return m_alternateEncoder.getPosition() * Constants.Climber.kInchesPerRotation;
    }

    public void setHeight(double height)  // For external height setting
    {
        // TODO - add safety checks
        m_pidController.setReference(height / Constants.Climber.kInchesPerRotation, CANSparkMax.ControlType.kPosition); // Sets internal PID to new height
        target = angle;
    }
}
