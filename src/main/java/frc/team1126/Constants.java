package frc.team1126;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.Toolbox.PIDGains;

import java.util.List;
public final class Constants {

  
public static class CANdleConstants
{
  /* CANdle ID */
public static final int CANdleID = 3;

/* Purple RGB */
public static final int PURPLE_R = 255;
public static final int PURPLE_G = 0;
public static final int PURPLE_B = 191;

/* Yellow RGB */
public static final int YELLOW_R = 255;
public static final int YELLOW_G = 130;
public static final int YELLOW_B = 0;

/* Red RGB */
public static final int RED_R = 255;
public static final int RED_G = 0;
public static final int RED_B = 0;

/* Green RGB */
public static final int GREEN_R = 0;
public static final int GREEN_G = 255;
public static final int GREEN_B = 0;

/* Blue RGB */
public static final int BLUE_R = 0;
public static final int BLUE_G = 0;
public static final int BLUE_B = 255;

}

  public static class LimelightConstants {
    public static final boolean USE_FOR_TARGETING = true;
    public static final boolean LED_ON_DEFAULT = false;
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

  public static final double SPEAKER_HEIGHT = 78.0; // height needs to be changed
		public static final double AMP_HEIGHT = 26.0;
		public static final double LIMELIGHT_HEIGHT = 44; // measured from bottom of bumper to bottom of limelight
		public static final double INITIAL_ANGLE = 0;
		public static final double VISION_ANGLE_TOLERANCE = 0;
		public static final double APRILTAG_PIPELINE = 0;
		public static final String LIMELIGHT_NAME = "";

		public static final String LIMELIGHT_TABLE_KEY = "limelight";
		public static final String HORIZONTAL_OFFSET = "tx";
		public static final String HAS_VALID_TARGETS = "tv";
		public static final String VERTICAL_OFFSET = "ty";
		public static final String TARGET_AREA = "ta";
		public static final String LIMELIGHT_SKEW = "ts";
		public static final String LED_MODE = "ledMode";
		public static final String CAM_MODE = "camMode";
}
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
  
    
    /** General robot constants */
    public static final class GeneralConstants {

      // Driver controller port
      public static final int DRIVER_CONTROLLER_ID = 0;

      // Operator controller port
      public static final int OPERATOR_CONTROLLER_ID = 1;

    }
  
    /** Constants revolving around swerve subsystem */
    public static class SwerveConstants {

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
     
      //for endgame rumble feature
      public static final int EndGameSeconds = 30;
      public static final int StopRumbleSeconds = 28;
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
    public static final class AutonConstants {
  
      public static final double MAX_VELOCITY = 3.0;
      public static final double MAX_ACCELERATION = 2.0;
      public static final PathConstraints CONSTRAINTS =
          new PathConstraints(AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION);
  
      public static final double XY_CONTROLLER_P = 4;
      public static final double THETA_CONTROLLER_P = 1;
    }
  }
  
