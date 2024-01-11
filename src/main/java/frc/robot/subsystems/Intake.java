package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase 
{
    private TalonSRX intakeLeft;
    private TalonSRX intakeRight;

    public Intake() 
    {
        intakeLeft = new TalonSRX(Constants.IntakeLeftID);
        intakeLeft.configPeakCurrentLimit(20, 10);          // 20A
        intakeLeft.configPeakCurrentDuration(200, 10);      // 200ms
        intakeLeft.configContinuousCurrentLimit(10, 10);    // 10A
        intakeLeft.enableCurrentLimit(true);
        intakeLeft.setInverted(true);     

        intakeRight = new TalonSRX(Constants.IntakeRightID);
        intakeRight.configPeakCurrentLimit(20, 10);          // 20A
        intakeRight.configPeakCurrentDuration(200, 10);      // 200ms
        intakeRight.configContinuousCurrentLimit(10, 10);    // 10A
        intakeRight.enableCurrentLimit(true);
        intakeRight.setInverted(false); // JTL 10-10-23 CAN CHANGE TO TRUE AND REMOVE "-" from values below
    }

    public void setSpeed(double speed)  // Auto / Tele
    {
        intakeLeft.set(ControlMode.PercentOutput, speed); 
        intakeRight.set(ControlMode.PercentOutput, -speed);
    }

    public void runIntake(Joystick opJoystick, Joystick drJoystick)    // Teleop
    {
        if (opJoystick.isConnected())   // OP Joystick IS Connected - Op / Driver Mode
        {
            if (opJoystick.getRawButton(XboxController.Button.kRightBumper.value))    // Cube / Cone Intake - Right Front Bumper - OP
            {
                intakeLeft.set(ControlMode.PercentOutput, 0.25); 
                intakeRight.set(ControlMode.PercentOutput, -0.25); 
            } 
            else if (opJoystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Cube / Cone Outtake - Left Front Bumper - OP
            {
                intakeLeft.set(ControlMode.PercentOutput, -1); 
                intakeRight.set(ControlMode.PercentOutput, 1); 
            } 
            else 
            {
                // Might need to add a line to set intake to cube by default when no buttons pressed
                intakeLeft.set(ControlMode.PercentOutput, 0);
                intakeRight.set(ControlMode.PercentOutput, 0);
                // TK 45 - 10-3-23 Probably need to set to slow constant IN to hold cube (PID won't cut it)(Also, no encoder) - instead just held intake button
            }
        }
        else    // Joystick NOT Connected - Driver ONLY Mode
        {
            if (drJoystick.getRawButton(6))    // Cube / Cone Intake - Right Front Bumper - DRIVER
            {
                intakeLeft.set(ControlMode.PercentOutput, 0.25); 
                intakeRight.set(ControlMode.PercentOutput, -0.25); 
            } 
            else if (drJoystick.getRawButton(5)) // Cube / Cone Outtake - Left Front Bumper - DRIVER
            {
                intakeLeft.set(ControlMode.PercentOutput, -1); 
                intakeRight.set(ControlMode.PercentOutput, 1); 
            } 
            else 
            {
                // Might need to add a line to set intake to cube by default when no buttons pressed
                intakeLeft.set(ControlMode.PercentOutput, 0);
                intakeRight.set(ControlMode.PercentOutput, 0);
                // TK 45 - 10-3-23 Probably need to set to slow constant IN to hold cube (PID won't cut it)(Also, no encoder) - instead just held intake button
            }
        }
    }
}
