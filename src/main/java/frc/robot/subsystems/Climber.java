package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        climber.setSmartCurrentLimit(40);

        m_alternateEncoder = climber.getAlternateEncoder(kAltEncType, kCPR);

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

        // PID coefficients // TODO - update values (TUNE)
        kP = 0.1;
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .2;        // TODO - Update this to 1 after testing
        kMinOutput = -.2;       // TODO - Update this to 1 after testing

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF); // Functionally the same as kV, just different units
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
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

        public void testModeCalibration()
    {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) 
        { 
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
    }
}
