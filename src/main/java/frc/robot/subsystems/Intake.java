package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

public class Intake extends SubsystemBase 
{
    // TODO - Add 1-2 photoelectric sensors to intake

    private CANSparkMax intake;
    private DigitalInput intakeSensor;

    public Intake() 
    {
        intake = new CANSparkMax(Constants.IntakeID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(Constants.IntakeSensor1Port);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        return !intakeSensor.get();
    }

    public void setSpeed(double speed)  // For external speed setting
    {
        // Speed Set: -1 to 1
        intake.set(speed/100);  // Converts speed percent to -1 to 1 range
    }

    public void runIntake(Joystick opJoystick, Joystick drJoystick)
    {
        //if button pressed -> run intake
        if(opJoystick.getRawButton(XboxController.Button.kRightBumper.value)) // TODO - Update button // Intake button pressed
        {
            intake.set(0.75);   // TODO - adjust speed and maybe direction
        }
        else    // Intake button not pressed
        {
            intake.set(0);
        }
    }

    public void autoIntake()
    {
        // TODO - Create intakeNote() in intake subsystem   // using sensors and vision and stuff
    }

    public void periodic()
    {
        SmartDashboard.putBoolean("Intake GamePiece Detected", intakeSensor.get());

    }
}
