package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm extends SubsystemBase
{
    private TalonSRX arm;
    private PIDController pidController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

    double target = 0;
    int angle;
    DigitalInput armSwitchForward = new DigitalInput(1); //limit switch that re-zeros the arm encoder when forward // Probably won't use
    //CANandcoder m_encoder;  // Use if CANanCoder doesn't work
    DutyCycleEncoder m_encoder;
    //PIDController pid;
    //double set_value;

    public Arm() 
    {
        arm = new TalonSRX(Constants.ArmID);
        arm.setInverted(false);
        m_encoder = new DutyCycleEncoder(0);  
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", getAngle());
    }
    

    public double getTarget() 
    {
        return target;
    }

    public double getAngle()
    {
        return m_encoder.getAbsolutePosition()*360-151.8;
    }

    //goto a preset

    public void setArmPreset(double target)
    {
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));
    }

    public void setAngle(double angle)  // Auto / Tele
    {
        arm.set(ControlMode.PercentOutput, pidController.calculate(getAngle(), angle));
        target = angle;
    }
    

    public void moveAngle(Joystick opJoystick, Joystick drJoystick)    // For Teleop
    {   
        // IF OP Joystick IS Connected:
        if(opJoystick.isConnected())
        {
            // Check Buttons
            if (opJoystick.getRawButton(XboxController.Button.kY.value))  // Y Button Pressed - MID FRONT SCORE - OP
            {
                target = Constants.ARM_MID_FRONT_SCORE; 
            }
            else if (opJoystick.getRawButton(XboxController.Button.kA.value))  // A Button Pressed - LOW FRONT SCORE - OP
            {
                target = Constants.ARM_LOW_FRONT_SCORE; 
            }
            else if (opJoystick.getRawButton(XboxController.Button.kX.value))   // X Button Pressed - STRAIGT UP - OP
            {
                target = Constants.ARM_TOP;
            }  
            else    // No Button Pressed
            {
                holdAngle();
            }
        }
        else    // OP Joystick NOT Connected
        {
            if (drJoystick.getRawButton(XboxController.Button.kY.value))  // Y Button Pressed - MID FRONT SCORE - DRIVER
            {
                target = Constants.ARM_MID_FRONT_SCORE; 
            }
            else if (drJoystick.getRawButton(XboxController.Button.kA.value))  // A Button Pressed - LOW FRONT SCORE - DRIVER
            {
                target = Constants.ARM_LOW_FRONT_SCORE; 
            }
            else if (opJoystick.getRawButton(XboxController.Button.kX.value))   // X Button Pressed - STRAIGT UP - OP
            {
                target = Constants.ARM_TOP;
            }  
            else    // No Button Pressed
            {
                holdAngle();
            }
        }
        // Move to target
        arm.set(ControlMode.PercentOutput, pidController.calculate(getAngle(), target));
    }
    

    public void holdAngle() // Maintains the current angle using PID.
    {
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));
    }
}