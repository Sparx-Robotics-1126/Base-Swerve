// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

import frc.lib.swervelib.FirstOrderSwerveDrive;
import frc.lib.swervelib.SecondOrderSwerveDrive;
import frc.lib.swervelib.SwerveController;
import frc.lib.swervelib.SwerveDrive;
import frc.lib.swervelib.SwerveModule;
import frc.team1126.Mechanisms.SwerveModules;
import frc.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.lib.swervelib.parser.SwerveDriveConfiguration;
import frc.lib.swervelib.parser.SwerveParser;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.team1126.Constants;
import frc.team1126.Constants.ModuleConstants;
import frc.team1126.Constants.ModuleConstants.BackLeftModule;
import frc.team1126.Constants.ModuleConstants.BackRightModule;
import frc.team1126.Constants.ModuleConstants.FrontLeftModule;
import frc.team1126.Constants.ModuleConstants.FrontRightModule;
import frc.team1126.Constants.ModuleConstants.GenericModuleConstants;
import frc.team1126.sensors.Limelight;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive       swerveDrive;
  
  private final SwerveModules frontLeft;
	private final SwerveModules frontRight;
	private final SwerveModules backLeft;
	private final SwerveModules backRight;
  
  private SwerveDrivePoseEstimator posEstimator;
  private SwerveModulePosition[] swervePositions;
  private final PigeonSubsystem gyro;


  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    frontLeft = new SwerveModules(
				"FL",
				FrontLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true,true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		frontRight = new SwerveModules(
				"FR",
				FrontRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true, true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backLeft = new SwerveModules(
				"RL",
				BackLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true,true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backRight = new SwerveModules(
				"RR",
				BackRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true, true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		swervePositions = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				backLeft.getPosition(),
				backRight.getPosition()
		};

   gyro = PigeonSubsystem.getInstance();//new WPI_Pigeon2(DriveConstants.kPigeonPort, Constants.kCTRECANBusName);
		gyro.setYaw(0);
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      if(Constants.DriveConstants.IS_FIRST_ORDER) swerveDrive = new SwerveParser(directory).createFirstOrderSwerveDrive();
      else                                        swerveDrive = new SwerveParser(directory).createSecondOrderSwerveDrive();
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    frontLeft = new SwerveModules(
				"FL",
				FrontLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true,true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		frontRight = new SwerveModules(
				"FR",
				FrontRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true, true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backLeft = new SwerveModules(
				"RL",
				BackLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true,true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backRight = new SwerveModules(
				"RR",
				BackRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				true, true,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		swervePositions = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				backLeft.getPosition(),
				backRight.getPosition()
    };
    gyro = PigeonSubsystem.getInstance();//new WPI_Pigeon2(DriveConstants.kPigeonPort, Constants.kCTRECANBusName);
		gyro.setYaw(0);
    if(Constants.DriveConstants.IS_FIRST_ORDER) swerveDrive = new FirstOrderSwerveDrive(driveCfg, controllerCfg);
    else                                        swerveDrive = new SecondOrderSwerveDrive(driveCfg, controllerCfg);
  }

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
//    SmartDashboard.putBoolean("Field Centric", m_fieldOriented);

    SmartDashboard.putNumber("LimeLight Distance", Limelight.getInstance().getDistance(3.5));

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

  public void resetPoseEstimator(Pose2d pose) {
		posEstimator.resetPosition(
		gyro.getRotation2d(), 
			swervePositions, 
			pose);
	}

  public Pose2d getPoseEstimatorPose2d() {
		return posEstimator.getEstimatedPosition();
	}

  public void setModuleStates(SwerveModuleState[] desiredStates) {
		setModuleStates(desiredStates, false);
	}

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isTurbo) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, GenericModuleConstants.kMaxModuleSpeedMetersPerSecond);

		frontLeft.setDesiredState(desiredStates[0], isTurbo);
		frontRight.setDesiredState(desiredStates[1], isTurbo);
		backLeft.setDesiredState(desiredStates[2], isTurbo);
		backRight.setDesiredState(desiredStates[3], isTurbo);
	}

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), true, 4);
  }
}