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
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
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

        // set PID coefficients
        m_pidController.setP(Constants.Shoulder.kP);
        m_pidController.setI(Constants.Shoulder.kI);
        m_pidController.setD(Constants.Shoulder.kD);
        m_pidController.setIZone(Constants.Shoulder.kIz);
        m_pidController.setFF(Constants.Shoulder.kFF);
        m_pidController.setOutputRange(-Constants.Shoulder.kMinOutput, Constants.Shoulder.kMaxOutput);
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
        if(opJoystick.getRawButton(XboxController.Button.kY.value))       // TODO - Update button // Shooter button pressed UP
        {
            setAngle(-45);   // TODO - update angle   // 45?   // Speaker shoot angle (Assuming -45 for feed location)
        }
        else if(opJoystick.getRawButton(XboxController.Button.kA.value))    // TODO - TODO - Update button // Shooter button pressed DOWN
        {
            setAngle(45);  // TODO - update angle    // -45?   // Amp shoot angle (Assuming -45 for feed location)
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
        return (getAngle() == target);
    }
}