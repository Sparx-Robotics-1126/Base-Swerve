// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

import frc.lib.swervelib.FirstOrderSwerveDrive;
import frc.lib.swervelib.SecondOrderSwerveDrive;
import frc.lib.swervelib.SwerveController;
import frc.lib.swervelib.SwerveDrive;
import frc.lib.swervelib.imu.SwerveIMU;
import frc.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.lib.swervelib.parser.SwerveDriveConfiguration;
import frc.lib.swervelib.parser.SwerveParser;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.team1126.Constants;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive       swerveDrive;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      if(Constants.SwerveConstants.IS_FIRST_ORDER) swerveDrive = new SwerveParser(directory).createFirstOrderSwerveDrive();
      else                                        swerveDrive = new SwerveParser(directory).createSecondOrderSwerveDrive();
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    if(Constants.SwerveConstants.IS_FIRST_ORDER) swerveDrive = new FirstOrderSwerveDrive(driveCfg, controllerCfg);
    else                                        swerveDrive = new SecondOrderSwerveDrive(driveCfg, controllerCfg);

    setupPathPlanner();
  }

  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(swerveDrive.getSwerveController().config.headingPIDF.p ,
                            swerveDrive.getSwerveController().config.headingPIDF.i,
                            swerveDrive.getSwerveController().config.headingPIDF.d),
                    // Rotation PID constants
                    4.5,
                    // Max module speed, in m/s
                    swerveDrive.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(),
                    // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig()
                    // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this // Reference to this subsystem to set requirements
    );  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }


  public SwerveIMU getSwerveIMU(){
    return swerveDrive.getSwerveIMU();
  }
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the gyro angle to zero and sets odometry to the same position, but facing toward offset.
   * 
   * @param offset Angle to offset gyro yaw in degrees.
   */
  public void offsetGyroYaw(double offset)
  {
    swerveDrive.offsetGyroYaw(offset);
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * 
   * Syntactic sugar of getYaw
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getYaw()
  {
    return swerveDrive.getYaw();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getRoll()
  {
    return swerveDrive.getRoll();
  }

     /**
   * Gets the current yaw velocity of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw velocity as an {@link Rotation2d} angular velocity
   */
  public Rotation2d getYawVel() {
    return swerveDrive.getYawVel();
  }

  /**
   * Gets the current pitch velocity of the robot, as reported by the imu.
   *
   * @return The pitch velocity as an {@link Rotation2d} angular velocity
   */
  public Rotation2d getPitchVel() {
    return swerveDrive.getPitchVel();
  }

  /**
   * Gets the current roll velocity of the robot, as reported by the imu.
   *
   * @return The roll velocity as an {@link Rotation2d} angular velocity
   */
  public Rotation2d getRollVel() {
    return swerveDrive.getRollVel();
  }
  public double getMaxAcceleration() {
    return 1.0;
}

public double getMaxAngularSpeed() {
    return Math.PI*2;
}

public double getMaxAngularAcceleration() {
    return Math.PI;
} 
  /**
   * Set the translational scalar for the swerve translational velocity
   * 
   * @param scalar New translational scalar between [0:1]
   */
  public void setTranslationalScalar(double scalar)
  {
    swerveDrive.setTranslationalScalar(scalar);
  }

  /**
   * Set the rotational scalar for the swerve rotational velocity
   * 
   * @param scalar New rotational scalar between [0:1]
   */
  public void setRotationalScalar(double scalar)
  {
    swerveDrive.setRotationalScalar(scalar);
  }

  /**
   * Set the heading angle for the swerve heading controller
   * 
   * @param angle Angle of the heading in radians
   */
  public void setHeadingAngle(double angle)
  {
    swerveDrive.setHeadingAngle(angle);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.getSwerveController();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), true, 4);
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart)
    {
      resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}