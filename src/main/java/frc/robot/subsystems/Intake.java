package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Intake extends SubsystemBase 
{
    private CANSparkMax intake;
    private DigitalInput intakeSensor;

    public Intake() 
    {
        intake = new CANSparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(Constants.Intake.IntakeSensor1Port);

        intake.setSmartCurrentLimit(40);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        return !intakeSensor.get();
    }

    public void setSpeed(double speed)  // For external speed setting
    {
        // Speed Set: -1 to 1
        intake.set(speed);  // Converts speed percent to -1 to 1 range
    }

    public void runIntake(Joystick opJoystick, Joystick drJoystick) // For manual control
    {
        //if button pressed -> run intake
        if(opJoystick.getRawButton(XboxController.Button.kRightBumper.value)) // Intake button pressed
        {
            intake.set(Constants.Intake.intakeSpeed);
        }
        else    // Intake button not pressed
        {
            intake.set(0);
        }
    }

    public void intakeUntilSeen()   // Intake until gamepiece is detected
    {
        if(!intakeSensor.get())        // If gamepiece is not detected
        {
            intake.set(Constants.Intake.intakeSpeed);    // TODO - Check direction
        }
        else if(intakeSensor.get())    // If gamepiece is detected
        {
            intake.set(0);
        }
        else
        {
            // ERROR
            // TODO - add error handling / output
        }
    }

    public void periodic()
    {
        SmartDashboard.putBoolean("Intake GamePiece Detected", intakeSensor.get());

    }
}
