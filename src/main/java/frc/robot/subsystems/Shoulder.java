package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Shoulder extends SubsystemBase
{
    private CANSparkMax shoulder;
    
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    double target = 0;
    int angle;

    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

    private SparkPIDController m_pidController;
    private static final int kCPR = 8192;   // Counts per revolution for REV Through Bore Encoder

    private RelativeEncoder m_alternateEncoder;

    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();

        m_alternateEncoder = shoulder.getAlternateEncoder(kAltEncType, kCPR);

        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = shoulder.getPIDController();

        /**
         * By default, the PID controller will use the Hall sensor from a NEO for its
         * feedback device. Instead, we can set the feedback device to the alternate
         * encoder object
         */
        m_pidController.setFeedbackDevice(m_alternateEncoder);

        // PID coefficients // TODO - UPDATE THESE VALUES
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", getAngle());
    }

    public double getAngle()
    {
        return m_alternateEncoder.getPosition(); // TODO - do math?
    }

    public void setAngle(double angle)  // Auto / Tele
    {
        m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition); // Sets internal PID to new position
        target = angle;
    }
}