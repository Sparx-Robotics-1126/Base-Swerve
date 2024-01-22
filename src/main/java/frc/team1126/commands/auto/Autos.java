// package frc.team1126.commands.auto;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.team1126.Constants;
// import frc.team1126.RobotContainer;

// import java.util.HashMap;

// public final class Autos {

//   private static final SendableChooser<Command> autonChooser = new SendableChooser<Command>();;
//   private static HashMap<String, Command> eventMap;
//   private static SwerveAutoBuilder autonBuilder;

//   public static void init() {

//     eventMap = new HashMap<>();
//     setMarkers();

//     autonBuilder =
//         new SwerveAutoBuilder(
//             RobotContainer.swerve::getPose,
//             RobotContainer.swerve::resetOdometry,
//             new PIDConstants(Constants.AutonConstants.XY_CONTROLLER_P, 0.0, 0.0),
//             new PIDConstants(Constants.AutonConstants.THETA_CONTROLLER_P, 0.0, 0.0),
//             RobotContainer.swerve::setChassisSpeeds,
//             eventMap,
//             true,
//             RobotContainer.swerve);

//     autonChooser.setDefaultOption("No-op", new InstantCommand());
//     autonChooser.addOption("Move 1 Meter", move1Meter());
//     autonChooser.addOption("Move 1 Meter Back", move1MeterBack());
//     SmartDashboard.putData("Auton Chooser", autonChooser);
//   }

//   private static void setMarkers() {
//     eventMap.put("Wait a Second", new WaitCommand(1)); //Example marker
//   }

//   public static Command getAutonomousCommand() {
//     return autonChooser.getSelected();
//   }

//   public static Command move1Meter() {
//     return autonBuilder.fullAuto(
//         PathPlanner.loadPathGroup(
//             "Move 1 Meter", Constants.AutonConstants.CONSTRAINTS));
//   }
//   public static Command move1MeterBack() {
//     return autonBuilder.fullAuto(
//         PathPlanner.loadPathGroup(
//             "Move 1 Meter Back", Constants.AutonConstants.CONSTRAINTS));
//   }
// }
