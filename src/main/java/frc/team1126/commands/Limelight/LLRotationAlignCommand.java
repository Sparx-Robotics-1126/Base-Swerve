package frc.team1126.commands.Limelight;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.team1126.commands.Limelight;

// import java.util.concurrent.TimeUnit;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.lib.Toolbox.DoubleSmoother;
// import frc.team1126.sensors.LimelightHelpers;
// import frc.team1126.RobotContainer;
// import frc.team1126.Constants.LimelightConstants;
// import frc.team1126.sensors.Limelight;
// import frc.team1126.subsystems.SwerveSubsystem;

// public class LLRotationAlignCommand extends CommandBase {

// 	private static SwerveSubsystem driveSubsystem;

// 	PIDController RotationPIDAmount;

// 	DoubleSmoother driveOutputSmoother;
// 	DoubleSmoother strafeOutputSmoother;


// 	Translation2d translation = new Translation2d();


// 	/** Creates a new LLTargetCommand. */
// 	public LLRotationAlignCommand() {

// 		driveSubsystem = RobotContainer.swerve;



// 		RotationPIDAmount = new PIDController(
// 				LimelightConstants.kLLAlignDriveGains.kP,
// 				LimelightConstants.kLLAlignDriveGains.kI,
// 				LimelightConstants.kLLAlignDriveGains.kD);
// 		RotationPIDAmount.setTolerance(0.5);

// 		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignDriveMotionSmoothing);

// 		// Use addRequirements() here to declare subsystem dependencies.
// 		addRequirements(driveSubsystem);
// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {

// 		LimelightHelpers.setLEDMode_ForceOn("");
// 		Limelight.getInstance().setForTargeting(true);
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {

// 		if (LimelightHelpers.getTV("")) {

// 			SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));

// 			double rotationAmount = RotationPIDAmount.calculate(LimelightHelpers.getTX(""), 0);

// 			//in last year's code, we had the x and y values as seperate parameters, replaced by translation2D parameter
// 			driveSubsystem.drive(new Translation2d(0,0), rotationAmount, true, true);

// 		} else {
// 			driveSubsystem.drive(new Translation2d(0,0), 0.0, true, true);
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
// 		return RotationPIDAmount.atSetpoint();
// 	}
// }
