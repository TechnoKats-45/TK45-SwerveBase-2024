package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Shoulder extends SubsystemBase
{
    private CANSparkMax shoulder;

    double target = 0;
    int angle;
    
    // RoboRio PID Implementation:
        //private double kP = .01, kI = 0.0001, kD = 0, kS, kG, kV, kA, feedForward;   // kp was .01
        //private double position = 0, velocity = 0, acceleration = 0;

        //private DutyCycleEncoder m_absoluteEncoder;
        //private PIDController m_pidController;
        //private ArmFeedforward m_feedforward;

    // Spark Max PID Implementation:
        private double kP = 0.1, kI =0, kD = 0, kIz = 0, kFF = 0;
        private SparkMaxAbsoluteEncoder m_absoluteEncoder;
        private SparkMaxPIDController m_pidController;


    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();
        shoulder.setSmartCurrentLimit(40);
        shoulder.setInverted(true);

        // RoboRio PID Implementation:
            //m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
            //m_pidController = new PIDController(kP, kI, kD);
            //m_absoluteEncoder = new DutyCycleEncoder(Constants.Shoulder.ShoulderEncoderPort);

        target = getAngle();    // TODO - remove this?

        // Spark Max PID Implementation:
        m_absoluteEncoder = shoulder.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_pidController = shoulder.getPIDController();
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(-Constants.Shoulder.maxSpeed, Constants.Shoulder.maxSpeed);
        m_pidController.setPositionPIDWrappingEnabled(false);   
        
        shoulder.burnFlash();
    }

    public double getAngle()
    {
        return m_absoluteEncoder.getPosition() * 360;
    }

    public void moveAngle(Joystick opJoystick, Joystick drJoystick) 
    {
        holdTarget();   // Hold target angle // Button readings should happen in RobotContainer
    }

    public void holdTarget() // Not needed with Spark MAX PID
    {
        // RoboRio PID Implementation:
            //double pidOutput = m_pidController.calculate(getAngle(), target);

            // Limit the speed to the range [-maxSpeed, maxSpeed]
            //double limitedSpeed = Math.max(-Constants.Shoulder.maxSpeed, Math.min(Constants.Shoulder.maxSpeed, pidOutput));

            // Set the motor speed with the limited value
            //shoulder.set(limitedSpeed);
    }

    public void setAlignedAngle(double z, double y, boolean tag)
    {
        if(tag) // If tag exists
        {
            setTarget((Constants.Shoulder.groundParallelAngle - (Math.tan(y/z) * (180 / Math.PI))));    // Calculates direct line to taget angle based on X and Z (Hypotenuse)
            SmartDashboard.putNumber("Calculated Shoulder Angle", (Math.tan(y/z) * (180 / Math.PI)));
            SmartDashboard.putNumber("Calculated - Offset", (Constants.Shoulder.groundParallelAngle - (Math.tan(y/z) * (180 / Math.PI))));  //  + Constants.Shoulder.groundParallelAngle
        }
        else
        {
            // TODO - add diagnostics
        }
    }

    public boolean isAligned()
    {
        if (Math.abs(getAngle() - target) <= 2) // If within 1 degree of target
        {
            // DO NOT PUT A PRINT STATEMENT HERE! - THIS WILL BREAK EVERYTHING... again
            return true;    
        } 
        else 
        {
            return false;
        }
    }

    public void setTarget(double setPoint)  // Assigns a new target angle
    {
        // Checks to make sure angle is within limits
        if(setPoint > Constants.Shoulder.maxAngle)
        {
            setPoint = Constants.Shoulder.maxAngle;
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        }
        else if(setPoint < Constants.Shoulder.minAngle)
        {
            setPoint = Constants.Shoulder.minAngle;
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        }
        else
        {
            //target = setPoint;
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        }
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Shoulder Angle", getAngle());
        SmartDashboard.putNumber("Shoulder Targer", target);
    }
}