package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private double target;

    public Intake() 
    {
        intake = new CANSparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(Constants.Intake.IntakeSensor1Port);

        intake.setSmartCurrentLimit(40);
    }

    public boolean detectGamePiece()   // Reads the sensor and returns true if game piece is detected
    {
        //SmartDashboard.putBoolean("Intake Sensor", !intakeSensor.get());
        return !intakeSensor.get();
    }

    public void setTarget(double speed)  // For external speed setting
    {
        target = speed;
    }

    public void holdTarget()    // For maintaining speed
    {
        intake.set(target);
        //SmartDashboard.putNumber("Intake Target", target);
    }

    public void runIntake(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setTarget(speed);
        holdTarget();
    }

    public void diagnostics()
    {
        SmartDashboard.putNumber("Intake Current", intake.getOutputCurrent());
        SmartDashboard.putNumber("Intake Voltage", intake.getBusVoltage());
        SmartDashboard.putBoolean("Intake Sensor", detectGamePiece());
    }
}
