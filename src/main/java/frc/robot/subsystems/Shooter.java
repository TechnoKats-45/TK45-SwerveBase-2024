package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax shooter;

    public Shooter()
    {
        shooter = new CANSparkMax(Constants.ShooterID, MotorType.kBrushless);
    }

    public void setSpeed(double speed)  // For external speed setting
    {
        // Speed Set: -1 to 1
        shooter.set(speed);
    }

    public void runShooter(Joystick opJoystick, Joystick drJoystick)
    {
        //if button pressed -> run shooter
        if(opJoystick.getRawButton(XboxController.Axis.kRightTrigger.value)) // TODO - Update button // Shooter button pressed  // TODO - add auto shoot checks, change button and mybe direction
        {
            shooter.set(1);
        }
        else    // Shooter button not pressed
        {
            shooter.set(0);
        }
    }
}
