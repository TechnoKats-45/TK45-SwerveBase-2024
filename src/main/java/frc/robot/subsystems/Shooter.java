package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax shooterBottom;
    private CANSparkMax shooterTop;

    public double kP = .0006, kI = 0, kD = 0.000001;
    //public double kP = 0, kI = 0, kD = 0;
    public double bkS = 0.07 / 12, bkV = 0.00018;
    public double tkS = 0.07 / 12, tkV = 0.00018;     // TODO - change tkS to tuned value
    private PIDController bottomPidController = new PIDController(kP, kI, kD);
    private PIDController topPidController = new PIDController(kP, kI, kD);
    private SimpleMotorFeedforward bottomMotorFeedforward = new SimpleMotorFeedforward(bkS, bkV);
    private SimpleMotorFeedforward topMotorFeedforward = new SimpleMotorFeedforward(tkS, tkV);

    private double NeoFreeSpeed = 5676; // RPM

    double target;
    double bottomSpeed;
    double topSpeed;

    private final RelativeEncoder shooterBottomEncoder;
    private final RelativeEncoder shooterTopEncoder;

    public Shooter()
    {
        shooterBottom = new CANSparkMax(Constants.Shooter.ShooterBottomID, MotorType.kBrushless);
        shooterBottom.setSmartCurrentLimit(60);
        shooterBottom.setIdleMode(IdleMode.kCoast);
        shooterBottomEncoder = shooterBottom.getEncoder();
        shooterBottom.setInverted(true);

        shooterTop = new CANSparkMax(Constants.Shooter.ShooterTopID, MotorType.kBrushless);
        shooterTop.setSmartCurrentLimit(60);
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTopEncoder = shooterTop.getEncoder();
        shooterTop.setInverted(true);
    }

    public double getSpeedBottom()
    {
        return shooterBottomEncoder.getVelocity();  // Returns RPM
    }

    public double getSpeedTop()
    {
        return shooterTopEncoder.getVelocity();  // Returns RPM
    }

    public void setTarget(double speed)
    {
        target = speed * NeoFreeSpeed;
    }

    public void holdTarget() 
    {        
        //SmartDashboard.putNumber("Bottom Calculated", bottomPidController.calculate(getSpeedBottom(), target) + bottomMotorFeedforward.calculate(target));
        shooterBottom.set(bottomPidController.calculate(getSpeedBottom(), target) + bottomMotorFeedforward.calculate(target));
        shooterTop.set(topPidController.calculate(getSpeedTop(), target) + topMotorFeedforward.calculate(target));
        SmartDashboard.putNumber("KP", kP);
    }

    public void runShooter(double speed) // sets and holds target speed - OPERATOR MANUAL CONTROL
    {
        setTarget(speed);
        holdTarget();
    }

    public void coastToZero()
    {
        shooterTop.set(0);
        shooterBottom.set(0);
    }

    public double getBottomCurrent()
    {
        return shooterBottom.getOutputCurrent();
    }

    public double getTopCurrent()
    {
        return shooterTop.getOutputCurrent();
    }

    public void diagnostics()
    {
        /*
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add("Shooter Speed", getSpeed());
        tab.add("Shooter Target", target);
        */
        SmartDashboard.putNumber("Bottom Shooter Current", getBottomCurrent());
        SmartDashboard.putNumber("RPM Target", target);
        SmartDashboard.putNumber("Bottom Shooter Speed", getSpeedBottom());
        SmartDashboard.putNumber("Bot RPM", shooterBottomEncoder.getVelocity());

        SmartDashboard.putNumber("Top Shooter Current", getTopCurrent());
        SmartDashboard.putNumber("Top Shooter Speed", getSpeedTop());
        SmartDashboard.putNumber("Top RPM", shooterTopEncoder.getVelocity());

        SmartDashboard.putNumber("Actual Voltage BOT", shooterBottom.getAppliedOutput());
        SmartDashboard.putNumber("Actual Voltage TOP", shooterTop.getAppliedOutput());
    }
}
