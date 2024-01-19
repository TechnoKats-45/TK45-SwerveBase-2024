package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Feeder extends SubsystemBase 
{
    private CANSparkMax feeder;

    public Feeder()
    {
        feeder = new CANSparkMax(Constants.FeederID, MotorType.kBrushless);
    }

    public void setSpeed(double speed)  // For external speed setting
    {
        // Speed Set: -1 to 1
        feeder.set(speed/100);  // Converts spped percent to -1 to 1 range
    }

    public void runFeeder(Joystick opJoystick, Joystick drJoystick)
    {
        //if button pressed -> run Feeder
        if(opJoystick.getRawButton(XboxController.Button.kA.value)) // TODO - Update button // Feeder button pressed
        {
            feeder.set(0.5);
        }
        else    // Feeder button not pressed
        {
            feeder.set(0);
        }
    }
}
