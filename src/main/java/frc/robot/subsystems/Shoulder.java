package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    double target = 0;
    int angle;

    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

    private SparkPIDController m_pidController;
    private static final int kCPR = 8192;   // Counts per revolution for REV Through Bore Encoder

    private RelativeEncoder m_alternateEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();
        shoulder.setSmartCurrentLimit(40);

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

        // PID coefficients // TODO - update values (TUNE)
        kP = 0.1;
        kI = 0;
        kD = 0
        
        ; 
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

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
    
    }

    public double getAngle()
    {
        return m_alternateEncoder.getPosition(); // TODO - do math? - hopefully not, depends on what Hex encoder returns
    }

    public void setAngle(double angle)  // For external angle setting
    {
        // TODO - add safety checks
        m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition); // Sets internal PID to new position
        target = angle;
    }

    public void moveAngle(Joystick opJoystick, Joystick drJoystick) // For manual control
    {
        //if button pressed -> go to shoulder preset
        if(opJoystick.getRawButton(XboxController.Button.kY.value))         // Shooter button pressed UP
        {
            setAngle(Constants.Shoulder.speakerScoreAngle);   // TODO - update angle   // 45?   // Speaker shoot angle (Assuming -45 for feed location)
        }
        else if(opJoystick.getRawButton(XboxController.Button.kA.value))    // Shooter button pressed DOWN
        {
            setAngle(Constants.Shoulder.ampScoreAngle);  // TODO - update angle    // -45?   // Amp shoot angle (Assuming -45 for feed location)
        }
        else
        {
            //  do nothing
        }
    }

    public void setAlignedAngle(double x, double z, boolean tag)
    {
        //double dist = Math.hypot(x, z); // Calculates direct line distance from target
        if(tag) // If tag exists
        {
            setAngle(Math.tan(z/x));    // Calculates angle to target based on X and Z Tangent (Opposite over Adjacent)
        }
    }

    public boolean isAligned()
    {
        if (Math.abs(getAngle() - target) <= 1) // If within 1 degree of target
        {
            return true;
        } 
        else 
        {
            return false;
        }
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