package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shoulder extends SubsystemBase
{
    private CANSparkMax shoulder;

    private double target = 0;
    private double difference;
    private int angle;

    // Was 0.01, increased to 0.015,
    public double kP = .015, kI = 0.00000001, kD = .00001, kS, kG, kV, kA, feedForward;   // Added I and D -JTL 12:12PM 3-10-24
    private DutyCycleEncoder m_absoluteEncoder;
    private PIDController m_pidController;


    public Shoulder() 
    {
        shoulder = new CANSparkMax(Constants.Shoulder.ShoulderID, MotorType.kBrushless);
        shoulder.restoreFactoryDefaults();
        shoulder.setSmartCurrentLimit(40);
        shoulder.setInverted(true);

        m_pidController = new PIDController(kP, kI, kD);
        m_pidController.disableContinuousInput();

        m_absoluteEncoder = new DutyCycleEncoder(Constants.Shoulder.ShoulderEncoderPort);

        target = getAngle();    // TODO - change to handoff angle eventually

    }

    public double getAngle()
    {
        return m_absoluteEncoder.getAbsolutePosition() * 360;
    }

    public void moveAngle(Joystick opJoystick, Joystick drJoystick) 
    {
        holdTarget();   // Hold target angle // Button readings should happen in RobotContainer
    }

    public void holdTarget() 
    {
        // Checks to make sure angle is within limits
        if(target >= Constants.Shoulder.maxAngle)
        {
            target = Constants.Shoulder.maxAngle;
        }
        else if(target <= Constants.Shoulder.minAngle)
        {
            target = Constants.Shoulder.minAngle;
        }
        else
        {
            // Do nothing
        }

        double pidOutput = m_pidController.calculate(getAngle(), target);

        // Limit the speed to the range [-maxSpeed, maxSpeed]
        double limitedSpeed = Math.max(-Constants.Shoulder.maxSpeed, Math.min(Constants.Shoulder.maxSpeed, pidOutput));

        // Set the motor speed with the limited value
        shoulder.set(limitedSpeed);
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
        difference = getAngle() - target;
        if (Math.abs(difference) <= 2) // If within 1 degree of target
        {
            return true;    
        } 
        else 
        {
            return false;
        }
    }

    public void setTarget(double setPoint)  // Assigns a new target angle   // TODO - somehow not catching negative numbers // I think this is now fixed - test JTL 2-28-24
    {
        // Checks to make sure angle is within limits
        if(setPoint >= Constants.Shoulder.maxAngle)
        {
            target = Constants.Shoulder.maxAngle;
        }
        else if(setPoint <= Constants.Shoulder.minAngle)
        {
            target = Constants.Shoulder.minAngle;
        }
        else
        {
            target = setPoint;
        }
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Shoulder Angle", getAngle());
        SmartDashboard.putNumber("Shoulder Targer", target);
    }
}