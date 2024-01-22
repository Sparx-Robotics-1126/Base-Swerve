package frc.team1126.commands.Limelight;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.team1126.commands.Limelight;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.team1126.Constants.LimelightConstants;
// import frc.team1126.sensors.LimelightHelpers;
// import frc.team1126.RobotContainer;
// import frc.team1126.subsystems.SwerveSubsystem;

// public class LLTargetCommand extends Command {

// 	private static SwerveSubsystem driveSubsystem;

// 	PIDController LLTargetpidController;

// 	Translation2d translation = new Translation2d();
// 	/** Creates a new LLTargetCommand. */
// 	public LLTargetCommand() {

// 		driveSubsystem = RobotContainer.swerve;

// 		LLTargetpidController = new PIDController(
// 				LimelightConstants.kLLTargetGains.kP,
// 				LimelightConstants.kLLTargetGains.kI,
// 				LimelightConstants.kLLTargetGains.kD);
// 		LLTargetpidController.setTolerance(0);

// 		// Use addRequirements() here to declare subsystem dependencies.
// 		addRequirements(driveSubsystem);
// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {
// 		double turnOutput = LLTargetpidController.calculate(LimelightHelpers.getTX(""), 0);
// 		driveSubsystem.drive(new Translation2d(0,0), turnOutput,true, true);
// 	}

// 	// Called once the command ends or is interrupted.
// 	@Override
// 	public void end(boolean interrupted) {
// 	}

// 	// Returns true when the command should end.
// 	@Override
// 	public boolean isFinished() {
// 		return LLTargetpidController.atSetpoint();
// 	}
// }
