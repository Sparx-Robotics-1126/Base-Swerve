//package frc.team1126.commands.auto;
//
////import com.pathplanner.lib.PathPlannerTrajectory;
////import com.pathplanner.lib.commands.PPSwerveControllerCommand;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.team1126.subsystems.SwerveSubsystem;
////import frc.robot.Constants.Auton;
////import frc.robot.subsystems.swervedrive.SwerveSubsystem;
//
//public class FollowTrajectory extends SequentialCommandGroup
//{
//
//    public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
//    {
//        addRequirements(drivebase);
//
//        if (resetOdometry)
//        {
//            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
//        }
//
//        addCommands(
//                new PPSwerveControllerCommand(
//                        trajectory,
//                        drivebase::getPose,
//                        Auton.xAutoPID.createPIDController(),
//                        Auton.yAutoPID.createPIDController(),
//                        Auton.angleAutoPID.createPIDController(),
//                        drivebase::setChassisSpeeds,
//                        drivebase)
//        );
//    }
//}