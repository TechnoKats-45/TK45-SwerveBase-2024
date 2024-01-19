package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);        // TODO - Update these values
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);             // TODO - Update these values
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);      // TODO - Update these values

    // TODO - Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot - TODO
    final double LINEAR_P = 0.1;    // TODO - Update these values
    final double LINEAR_D = 0.0;    // TODO - Update these values
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;   // TODO - Update these values
    final double ANGULAR_D = 0.0;   // TODO - Update these values
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    double rotationSpeed;

    public Vision()
    {
        
    }

    public void findHorizontalAngleOffset(Joystick opJoystick, Joystick drJoystick) // Finds the horizontal angle that we need to be set to, to be alligned with the target
    {

        if (opJoystick.getRawButton(XboxController.Button.kA.value)) // TODO - update button config // If button pressed, enter vision-alignment mode
        {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets())   // If we have targets, calculate the angular turn power
            {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);  // TODO - need to tell it what April Tag to aim at
            }
            else // If we have no targets, stay still
            {
                rotationSpeed = 0;
            }
        }
        else // If button not pressed, manual driver mode
        {
            rotationSpeed = drJoystick.getRawAxis(XboxController.Axis.kLeftX.value); // TODO - Update axis config ??? // TODO - this conflicts with the drive code???
        }
    }

    public void searchForTarget()   // Determine if a target is in view
    {

    }
}