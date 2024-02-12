package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import 

public class LEDs extends SubsystemBase 
{
    // TODO - Add LED initialization

    double target = 0;
    double speed;

    public LEDs()
    {
        if(s_Swerve.isAligned() && s_Limelight.isAligned)
        {
            setLEDs(0x00FF00);
        }
        else
        {
            setLEDs(0xFF0000);
        }
    }
}
