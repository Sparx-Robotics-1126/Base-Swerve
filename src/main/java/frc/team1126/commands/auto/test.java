// package frc.team1126.commands.auto;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;

// // import com.pathplanner.lib.PathConstraints;
// // import com.pathplanner.lib.PathPlanner;
// // import com.pathplanner.lib.PathPlannerTrajectory;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.team1126.subsystems.SwerveSubsystem;



// public class test extends SequentialCommandGroup{
// 	private PathPlannerTrajectory m_newPath;
//     public test(SwerveSubsystem drive) {

//  PathPlannerPath m_newPath = PathPlannerPath.fromPathFile("Example Path");
// 		// m_newPath = PathPlanner.loadPath("Leave Short Rotate", new PathConstraints(1.75, 2));
//         addCommands(drive.followTrajectoryCommand(m_newPath, true));
//         // PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Test1",
//         //         new PathConstraints(.2, 1));

//         // //Places Cube in Mid Node
//         // addCommands(new ReachToPosition(reachSubsystem, ReachSubsystem.reachPositions.CONE_MID)
//         //         .alongWith(new ElevationToPosition(elevationSubsystem, ElevationSubsystem.elevationPositions.CONE_MID)));
//         // addCommands(new WaitCommand(2.5));
//         // addCommands(new InstantCommand(() -> grabberSubsystem.grabberOpen()));
//         // addCommands(new SequentialCommandGroup(new ReachToPosition( reachSubsystem, ReachSubsystem.reachPositions.HOME),
//         //         new ElevationToPosition(elevationSubsystem, ElevationSubsystem.elevationPositions.HOME)));
//         // addCommands(new WaitCommand(.5));
//         // addCommands(new InstantCommand(() -> grabberSubsystem.grabberClose()));
//         addCommands(drive.followTrajectoryCommand(m_newPath, false));
//         //Drives Backwards over charge station, balances
// //        addCommands(new DriveToPitch(drive, 0.6, -10, true, true));
// //    // // drive until ramp starts going down
// //        addCommands(new DriveDistance(drive, -1.38, .5));
// //        addCommands(new WaitCommand(.8));
// //    // drive backwards for distance hopefully on ramp
// //        addCommands(new DriveToPitch(drive, .55, 10, false, false));
// //        // addCommands(new WaitCommand(.25));
// //        addCommands(new DriveDistance(drive, .35, .5));
// //        addCommands(new BalanceOnChargeStation(drive));

    
//     }
// }
