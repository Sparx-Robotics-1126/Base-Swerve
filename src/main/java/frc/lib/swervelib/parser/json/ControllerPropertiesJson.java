package frc.lib.swervelib.parser.json;

import frc.lib.swervelib.parser.PIDFConfig;
import frc.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.lib.swervelib.parser.SwerveDriveConfiguration;

/**
 * {@link swervelib.SwerveController} parsed class. Used to access the JSON data.
 */
public class ControllerPropertiesJson
{

  /**
   * The minimum radius of the angle control joystick to allow for heading adjustment of the robot.
   */
  public double     angleJoystickRadiusDeadband;
  /**
   * The PID used to control the robot heading.
   */
  public PIDFConfig heading;
    /**
   * The PID used to control the robot snap heading.
   */
  public PIDFConfig snap;

  /**
   * Create the {@link SwerveControllerConfiguration} based on parsed and given data.
   *
   * @param driveConfiguration {@link SwerveDriveConfiguration} parsed configuration.
   * @return {@link SwerveControllerConfiguration} object based on parsed data.
   */
  public SwerveControllerConfiguration createControllerConfiguration(
      SwerveDriveConfiguration driveConfiguration)
  {
    return new SwerveControllerConfiguration(
        driveConfiguration, heading, snap, angleJoystickRadiusDeadband);
  }
}
