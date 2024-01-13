package frc.team1126;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Toolbox.PIDGains;
import frc.lib.Toolbox.SwerveConstants;
import frc.lib.Toolbox.SwerveModuleConstants;
import frc.team1126.commands.Limelight.LLAlignCommand;

import java.util.HashMap;
import java.util.List;
public final class Constants {

    /* Field related constants */
    public static final class FieldConstants {
      // List of possible scoring locations as Pose2d objects
      public static final List<Pose2d> SCORING_POSITIONS =
          List.of(
              new Pose2d(
                  new Translation2d(0.555, 7.436),
                  Rotation2d.fromRadians(Math.PI)), // Red loading double station
              new Pose2d(new Translation2d(0.555, 6.146), Rotation2d.fromRadians(Math.PI)),
              new Pose2d(
                  new Translation2d(15.03, 5.061),
                  Rotation2d.fromDegrees(0.0)), // Red node scoring locations
              new Pose2d(new Translation2d(15.03, 4.405), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 3.846), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 3.298), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 2.74), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 2.2), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 1.62), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 1.06), Rotation2d.fromDegrees(0.0)),
              new Pose2d(new Translation2d(15.03, 0.52), Rotation2d.fromDegrees(0.0)),
              new Pose2d(
                  new Translation2d(15.64, 7.430),
                  Rotation2d.fromDegrees(0.0)), // Blue loading double substation
              new Pose2d(new Translation2d(15.64, 6.16), Rotation2d.fromDegrees(0.0)),
              new Pose2d(
                  new Translation2d(1.598, 4.996),
                  Rotation2d.fromRadians(-Math.PI)), // Blue node scoring locations
              new Pose2d(new Translation2d(1.598, 4.373), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 3.85), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 3.3), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 2.75), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 2.2), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 1.63), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 1.05), Rotation2d.fromRadians(-Math.PI)),
              new Pose2d(new Translation2d(1.598, 0.5), Rotation2d.fromRadians(-Math.PI)));
    }

    public static class ModuleConstants {

      public static final boolean kUseThroughBore = false;
      
  
  
      // gains set for R1 SDS mk4i using dual neo motors
      public static final PIDGains kModuleDriveGains = new PIDGains(.15, 0.001, 0);
      public static final PIDGains kModuleTurningGains = new PIDGains(1.5, 0, 0.0016);
  
      public static final class GenericModuleConstants {
        // Current limits for the wheels
        public static final int kTurnMotorCurrentLimit = 25;
        public static final int kDriveMotorCurrentLimit = 35;
  
        // Constants set for the _SDS MK4i_
        public static final double kTurnGearRatio = 1d / (150d / 7d);
        public static final double kDriveGearRatio = 1d / 8; //5.56;
        public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
  
        // The max speeds the modules are capable of
        public static final double kMaxModuleAccelMetersPerSecond = 4;
        public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);// 5.6;
  
        // Retune feedforward values for turning
        // public static final double kvTurning = .43205;
        // public static final double ksTurning = .17161; // Tuned February 2, 2023
  
        public static final double kDriveFeedForward = .2;
  
        public static final SwerveConstants kSwerveConstants = new SwerveConstants(
          kTurnMotorCurrentLimit, 
          kDriveMotorCurrentLimit, 
          kTurnGearRatio, 
          kDriveGearRatio, 
          kWheelCircumference, 
          kMaxModuleAccelMetersPerSecond, 
          kMaxModuleSpeedMetersPerSecond, 
          kDriveFeedForward);
      }
  
      // module specific constants
      public static final class FrontLeftModule {
        public static final int kTurningMotorID = 11;
        public static final int kLeaderDriveMotorID = 10;
        // public static final int kFollowerDriveMotorID = 9;
        public static final int kAbsoluteEncoderID = 12;
        public static final double kAngleOffset =31.992 +180;
        public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
            kAbsoluteEncoderID,
            kTurningMotorID,
            kLeaderDriveMotorID,
            // kFollowerDriveMotorID,
            kAngleOffset);
      }
  
      public static final class FrontRightModule {
        public static final int kTurningMotorID = 21;
        public static final int kLeaderDriveMotorID = 20;
        // public static final int kFollowerDriveMotorID = 10;
        public static final int kAbsoluteEncoderID = 22;
        public static final double kAngleOffset = 351.650 +180;
        public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
            kAbsoluteEncoderID,
            kTurningMotorID,
            kLeaderDriveMotorID,
            // kFollowerDriveMotorID,
            kAngleOffset);
      }
  
      public static final class BackLeftModule {
        public static final int kTurningMotorID = 41;
        public static final int kLeaderDriveMotorID = 40;
        // public static final int kFollowerDriveMotorID = 11;
        public static final int kAbsoluteEncoderID = 42;
        public static final double kAngleOffset =137.988 +180;
        public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
            kAbsoluteEncoderID,
            kTurningMotorID,
            kLeaderDriveMotorID,
            // kFollowerDriveMotorID,
            kAngleOffset);
      }
  
      public static final class BackRightModule {
        public static final int kTurningMotorID = 31;
        public static final int kLeaderDriveMotorID = 30;
        // public static final int kFollowerDriveMotorID = 12;
        public static final int kAbsoluteEncoderID = 32;
        public static final double kAngleOffset = 12.920 +180;
        public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
            kAbsoluteEncoderID,
            kTurningMotorID,
            kLeaderDriveMotorID,
            // kFollowerDriveMotorID,
            kAngleOffset);
      }
    }
  
    /** General robot constants */
    public static final class GeneralConstants {

      // Driver controller port
      public static final int DRIVER_CONTROLLER_ID = 0;

      // Operator controller port
      public static final int OPERATOR_CONTROLLER_ID = 1;

    }
  
    /** Constants revolving around swerve subsystem */
    public static class DriveConstants {

      // Joystick axis deadband for the swerve drive
      public static final double SWERVE_DEADBAND = 0.1;

      // Swerve default translational scalar
      public static final double SWERVE_NORMAL_TRANSLATION = 0.6;

      // Swerve slow translational scalar
      public static final double SWERVE_SLOW_TRANSLATION = 0.25;
          
      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 10;

      public static final boolean IS_FIRST_ORDER = true;

      public static final double DT_CONSTANT = 0.1;

      public static final boolean HEADING_CORRECTION = false;

      public static final boolean CHASSIS_VELOCITY_CORRECTION = false;
      
      public static final int kPigeonPort = 4;

      public static final double kTrackWidth = Units.inchesToMeters(20.75); // in meters!
		  public static final double kWheelBase = Units.inchesToMeters(20.75); // in meters!

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR
    }
  
    /** Constants revolving around the vision subsystem. */
    public static final class VisionConstants {
      // Camera name
      public static final String CAMERA_NAME = "OV5647";
  
      // Robot to camera transform
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
              new Rotation3d(0.0, 0.0, 0.0));
    }
  
    /** Constants revolving around auton modes. */
    public static final class AutoConstants {
      public static class PathPLannerConstants {

        // PID constants for path planner (these control drive direction not reaching
        // target wheel speeds)
        public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
        public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);
  
        public static final double kPPMaxVelocity = 2.00;
        public static final double kPPMaxAcceleration = 1.00;
  
        public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
          {
            put("TargetTape", new LLAlignCommand(false));
            put("TargetTag", new LLAlignCommand(true));
          }
        };
      }
      public static final double MAX_VELOCITY = 3.0;
      public static final double MAX_ACCELERATION = 2.0;
      public static final PathConstraints CONSTRAINTS =
          new PathConstraints(AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION);
  
      public static final double XY_CONTROLLER_P = 4;
      public static final double THETA_CONTROLLER_P = 1;
    }

    public static class LimelightConstants {
			public static final boolean USE_FOR_TARGETING = true;
			public static final boolean LED_ON_DEFAULT = true;
			public static final double CAMERA_MIN_FLOOR_HEIGHT = 113.03;
			public static final double CAMERA_INITIAL_PITCH = 0;
		// declare ID's of pipelines here
		public static final int kCubePipeline = 0;
		public static final int kReflectivePipeline = 1;
		public static final int kApriltagPipeline = 4;

		// PID values for limelight
		public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); // .008
		public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double kPuppyTurnMotionSmoothing = 0.3;
		public static final double kPuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
		public static final double kAlignDriveMotionSmoothing = 0;
		public static final double kAlignStrafeMotionSmoothing = 0;
	}
  }
  