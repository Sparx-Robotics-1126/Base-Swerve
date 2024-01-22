package frc.team1126.commands.Limelight;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.team1126.commands.Limelight;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.team1126.RobotContainer;
// import frc.team1126.Constants.LimelightConstants;
// import frc.lib.Toolbox.DoubleSmoother;
// import frc.team1126.sensors.LimelightHelpers;
// import frc.team1126.subsystems.SwerveSubsystem;
// public class LLPuppydogCommand extends Command {

// 	private static SwerveSubsystem driveSubsystem;

// 	ProfiledPIDController LLTargetpidController;
// 	PIDController LLDrivepidController;

// 	DoubleSmoother driveOutputSmoother;
// 	DoubleSmoother turnOutputSmoother;

// 	/** Creates a new LLTargetCommand. */
// 	public LLPuppydogCommand() {
		
// 		driveSubsystem = RobotContainer.swerve;

// 		LLTargetpidController = new ProfiledPIDController(
// 				LimelightConstants.kLLPuppyTurnGains.kP,
// 				LimelightConstants.kLLPuppyTurnGains.kI,
// 				LimelightConstants.kLLPuppyTurnGains.kD,
// 				new TrapezoidProfile.Constraints(40, 40));

// 		LLDrivepidController = new PIDController(
// 				LimelightConstants.kLLPuppyDriveGains.kP,
// 				LimelightConstants.kLLPuppyDriveGains.kI,
// 				LimelightConstants.kLLPuppyDriveGains.kD);

// 		turnOutputSmoother = new DoubleSmoother(LimelightConstants.kPuppyTurnMotionSmoothing);
// 		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kPuppyDriveMotionSmoothing);

// 		// Use addRequirements() here to declare subsystem dependencies.
// 		addRequirements(driveSubsystem);
// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {
// 		LimelightHelpers.setLEDMode_ForceOn("");
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {
// 		if (LimelightHelpers.getTV("")) {
// 			double turnPIDOutput = LLTargetpidController.calculate(LimelightHelpers.getTX(""), 0);
// 			double drivePIDOutput = LLDrivepidController.calculate(LimelightHelpers.getTY(""), .5);

// 			double turnOutput = turnOutputSmoother.smoothInput(turnPIDOutput);
// 			double driveOutput = driveOutputSmoother.smoothInput(drivePIDOutput);

// 			//I didn't know whether or not we still wanted to do robot centric drive, so i left these here
// 			driveSubsystem.robotCentricDrive(driveOutput, 0, -turnOutput);
// 		} else {
// 			driveSubsystem.robotCentricDrive(0, 0, 0);
// 		}
// 	}

// 	// Called once the command ends or is interrupted.
// 	@Override
// 	public void end(boolean interrupted) {
// 		LimelightHelpers.setLEDMode_PipelineControl("");
// 	}

// 	// Returns true when the command should end.
// 	@Override
// 	public boolean isFinished() {
// 		// return LLTargetpidController.atSetpoint();
// 		return false;
// 	}
// }
