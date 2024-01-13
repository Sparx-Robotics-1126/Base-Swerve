package frc.lib.swervelib;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Swerve Drive class representing and controlling the swerve drive.
 */
public abstract class SwerveDrive
{

  /**
   * The primary method for controlling the drivebase. Takes a Translation2d and a rotation rate, and calculates and
   * commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation       {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                          second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                          torwards port (left). In field-relative mode, positive x is away from the alliance wall
   *                          (field North) and positive y is torwards the left wall when looking through the driver
   *                          station glass (field West).
   * @param rotation          Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                          relativity.
   * @param fieldRelative     Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop        Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public abstract void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop);

  /**
   * Set the maximum speeds for desaturation.
   *
   * @param attainableMaxModuleSpeedMetersPerSecond         The absolute max speed that a module can reach in meters per
   *                                                        second.
   * @param attainableMaxTranslationalSpeedMetersPerSecond  The absolute max speed that your robot can reach while
   *                                                        translating in meters per second.
   * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can reach while rotating in
   *                                                        radians per second.
   */
  public abstract void setMaximumSpeeds(
      double attainableMaxModuleSpeedMetersPerSecond,
      double attainableMaxTranslationalSpeedMetersPerSecond,
      double attainableMaxRotationalVelocityRadiansPerSecond);

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto pathing.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  protected abstract void setRawModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop);

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto paths.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public abstract void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop);

  /**
   * Set chassis speeds with closed-loop velocity control and second order kinematics.
   *
   * @param chassisSpeeds Chassis speeds to set.
   */
  public abstract void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public abstract Pose2d getPose();

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public abstract ChassisSpeeds getFieldVelocity();

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public abstract ChassisSpeeds getRobotVelocity();
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param pose The pose to set the odometry to
   */
  public abstract void resetOdometry(Pose2d pose);

  /**
   * Post the trajectory to the field
   *
   * @param trajectory the trajectory to post.
   */
  public abstract void postTrajectory(Trajectory trajectory);

  /**
   * Gets the current module states (azimuth and velocity)
   *
   * @return A list of SwerveModuleStates containing the current module states
   */
  public abstract SwerveModuleState[] getStates();

  /**
   * Gets the current module positions (azimuth and wheel position (meters)). Inverts the distance from each module if
   * {@link #invertOdometry} is true.
   *
   * @return A list of SwerveModulePositions containg the current module positions
   */
  public abstract SwerveModulePosition[] getModulePositions();

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public abstract void zeroGyro();

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw as a {@link Rotation2d} angle
   */
  public abstract Rotation2d getYaw();

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public abstract Rotation2d getPitch();

  /**
   * Gets the current roll angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public abstract Rotation2d getRoll();

     /**
   * Gets the current yaw velocity of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw velocity as an {@link Rotation2d} angular velocity
   */
  public abstract Rotation2d getYawVel();

  /**
   * Gets the current pitch velocity of the robot, as reported by the imu.
   *
   * @return The pitch velocity as an {@link Rotation2d} angular velocity
   */
  public abstract Rotation2d getPitchVel();

     /**
   * Gets the current roll velocity of the robot, as reported by the imu.
   *
   * @return The roll velocity as an {@link Rotation2d} angular velocity
   */
  public abstract Rotation2d getRollVel();

  /**
   * Gets the current gyro {@link Rotation3d} of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public abstract Rotation3d getGyroRotation3d();

  /**
   * Gets current acceleration of the robot in m/s/s. If gyro unsupported returns empty.
   *
   * @return acceleration of the robot as a {@link Translation3d}
   */
  public abstract Optional<Translation3d> getAccel();

  /**
   * Gets current angular velocity of the robot in rad/s. If gyro unsupported returns empty.
   *
   * @return angular velocity of the robot as a {@link Rotation3d}
   */
  public abstract Optional<Rotation3d> getAngularVel();

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public abstract void setMotorIdleMode(boolean brake);

  /**
   * Set the maximum speed of the drive motors, modified {@link SwerveControllerConfiguration#maxSpeed} and
   * {@link SwerveDriveConfiguration#maxSpeed} which is used for the
   * {@link FirstOrderSwerveDrive#setRawModuleStates(SwerveModuleState2[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates.
   *
   * @param maximumSpeed            Maximum speed for the drive motors in meters / second.
   * @param updateModuleFeedforward Update the swerve module feedforward to account for the new maximum speed. This
   *                                should be true unless you have replaced the drive motor feedforward with
   *                                {@link FirstOrderSwerveDrive#replaceSwerveModuleFeedforward(SimpleMotorFeedforward)}
   */
  public abstract void setMaximumSpeed(double maximumSpeed, boolean updateModuleFeedforward);

  /**
   * Set the maximum speed of the drive motors, modified {@link SwerveControllerConfiguration#maxSpeed} and
   * {@link SwerveDriveConfiguration#maxSpeed} which is used for the
   * {@link FirstOrderSwerveDrive#setRawModuleStates(SwerveModuleState2[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates. Overwrites the
   * {@link SwerveModule#feedforward}.
   *
   * @param maximumSpeed Maximum speed for the drive motors in meters / second.
   */
  public abstract void setMaximumSpeed(double maximumSpeed);

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public abstract void lockPose();

  /**
   * Get the swerve module poses and on the field relative to the robot.
   *
   * @param robotPose Robot pose.
   * @return Swerve module poses.
   */
  public abstract Pose2d[] getSwerveModulePoses(Pose2d robotPose);

  /**
   * Setup the swerve module feedforward.
   *
   * @param feedforward Feedforward for the drive motor on swerve modules.
   */
  public abstract void replaceSwerveModuleFeedforward(SimpleMotorFeedforward feedforward);

  /**
   * Update odometry should be run every loop. Synchronizes module absolute encoders with relative encoders
   * periodically. In simulation mode will also post the pose of each module. Updates SmartDashboard with module encoder
   * readings and states.
   */
  public abstract void updateOdometry();

  /**
   * Synchronize angle motor integrated encoders with data from absolute encoders.
   */
  public abstract void synchronizeModuleEncoders();

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose       Robot {@link Pose2d} as measured by vision.
   * @param timestamp       Timestamp the measurement was taken as time since startup, should be taken from
   *                        {@link Timer#getFPGATimestamp()} or similar sources.
   * @param soft            Add vision estimate using the
   *                        {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} function, or hard
   *                        reset odometry with the given position with
   *                        {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry#resetPosition(Rotation2d,
   *                        SwerveModulePosition[], Pose2d)}.
   * @param trustWorthiness Trust level of vision reading when using a soft measurement, used to multiply the standard
   *                        deviation. Set to 1 for full trust.
   */
  public abstract void addVisionMeasurement(Pose2d robotPose, double timestamp, boolean soft, double trustWorthiness);

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getFPGATimestamp()} or similar sources.
   * @param soft                     Add vision estimate using the
   *                                 {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} function, or
   *                                 hard reset odometry with the given position with
   *                                 {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry#resetPosition(Rotation2d,
   *                                 SwerveModulePosition[], Pose2d)}.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.
   */
  public abstract void addVisionMeasurement(Pose2d robotPose, double timestamp, boolean soft,
                                   Matrix<N3, N1> visionMeasurementStdDevs);


  /**
   * Set the expected gyroscope angle using a {@link Rotation3d} object. To reset gyro, set to a new
   * {@link Rotation3d}.
   *
   * @param gyro expected gyroscope angle.
   */
  public abstract void setGyro(Rotation3d gyro);

  /**
   * Helper function to get the {@link FirstOrderSwerveDrive#swerveController} for the {@link FirstOrderSwerveDrive} which can be used to
   * generate {@link ChassisSpeeds} for the robot to orient it correctly given axis or angles, and apply
   * {@link edu.wpi.first.math.filter.SlewRateLimiter} to given inputs. Important functions to look at are
   * {@link SwerveController#getTargetSpeeds(double, double, double, double)},
   * {@link SwerveController#addSlewRateLimiters(SlewRateLimiter, SlewRateLimiter, SlewRateLimiter)},
   * {@link SwerveController#getRawTargetSpeeds(double, double, double)}.
   *
   * @return {@link SwerveController} for the {@link FirstOrderSwerveDrive}.
   */
  public abstract SwerveController getSwerveController();

  /**
   * Get the {@link SwerveModule}s associated with the {@link FirstOrderSwerveDrive}.
   *
   * @return {@link SwerveModule} array specified by configurations.
   */
  public abstract SwerveModule[] getModules();

   /**
   * Set the translational scalar for the swerve translational velocity
   * 
   * @param scalar New translational scalar between [0:1]
   */
  public abstract void setTranslationalScalar(double scalar);

  /**
   * Set the rotational scalar for the swerve rotational velocity
   * 
   * @param scalar New rotational scalar between [0:1]
   */
  public abstract void setRotationalScalar(double scalar);


  /**
   * Set the heading angle for the swerve heading controller
   * 
   * @param angle Angle of the heading in radians
   */
  public abstract void setHeadingAngle(double angle);

  /**
   * Sets the gyro angle to zero and sets odometry to the same position, but facing toward offset.
   * 
   * @param offset Angle to offset gyro yaw in degrees.
   */
  public abstract void offsetGyroYaw(double offset);

  /**
   * Reset the drive encoders on the robot, useful when manually resetting the robot without a reboot, like in
   * autonomous.
   */
  public abstract void resetEncoders();

  /**
   * Enable second order kinematics for simulation and modifying the feedforward. Second order kinematics could increase
   * accuracy in odometry.
   *
   * @param moduleFeedforward Module feedforward to apply should be between [-1, 1] excluding 0.
   */
  public abstract void enableSecondOrderKinematics(double moduleFeedforward);

  /**
   * Enable second order kinematics for tracking purposes but completely untuned.
   */
  public abstract void enableSecondOrderKinematics();

  /**
   * Disable second order kinematics.
   */
  public abstract void disableSecondOrderKinematics();

}
