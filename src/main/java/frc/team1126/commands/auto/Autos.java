//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.team1126.commands.auto;
//
////import com.pathplanner.lib.PathConstraints;
////import com.pathplanner.lib.PathPlanner;
////import com.pathplanner.lib.PathPlannerTrajectory;
////import com.pathplanner.lib.PathPoint;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
////import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.*;
//import frc.team1126.subsystems.SwerveSubsystem;
//import frc.team1126.Constants.Auton;
////import frc.robot.subsystems.swervedrive.SwerveSubsystem;
//
//public final class Autos
//{
//
//    private Autos()
//    {
//        throw new UnsupportedOperationException("This is a utility class!");
//    }
//
//    /**
//     * Example static factory for an autonomous command.
//     */
//    public static Command exampleAuto(SwerveSubsystem swerve)
//    {
//        PathPlannerTrajectory example = PathPlannerPath.fromPathFile("AdvancedBlueAuto2",
//                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
//        // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
////    PathPlannerTrajectory example = PathPlanner.generatePath(
////        new PathConstraints(4, 3),
////        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
////// position, heading(direction of travel), holonomic rotation
////        new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
////// position, heading(direction of travel), holonomic rotation
////        new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90))
////        // position, heading(direction of travel), holonomic rotation
////                                                            );
//        return Commands.sequence(new FollowTrajectory(swerve, example, true));
//    }
//
//
//}