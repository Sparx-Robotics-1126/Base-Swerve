//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.team1126.commands.Limelight;
//
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.lib.Toolbox.DoubleSmoother;
//import frc.team1126.RobotContainer;
//import frc.team1126.Constants.LimelightConstants;
//
//import frc.team1126.subsystems.SwerveSubsystem;
//import frc.team1126.subsystems.sensors.Limelight;
//import frc.team1126.subsystems.sensors.LimelightHelpers;
//
//public class LLAlignCommand extends Command {
//
//	private static SwerveSubsystem driveSubsystem;
//
//	PIDController StrafePIDController;
//	PIDController DrivePIDController;
//
//	DoubleSmoother driveOutputSmoother;
//	DoubleSmoother strafeOutputSmoother;
//
//	Translation2d translation = new Translation2d(0,0);
//
//	boolean targetTags;
//
//	/** Creates a new LLTargetCommand. */
//	public LLAlignCommand(boolean targetTags) {
//
//		driveSubsystem = RobotContainer.drivebase;
//
//		this.targetTags = targetTags;
//
//		StrafePIDController = new PIDController(
//				LimelightConstants.kLLAlignStrafeGains.kP,
//				LimelightConstants.kLLAlignStrafeGains.kI,
//				LimelightConstants.kLLAlignStrafeGains.kD);
//		StrafePIDController.setTolerance(0.25);
//
//		DrivePIDController = new PIDController(
//				LimelightConstants.kLLAlignDriveGains.kP,
//				LimelightConstants.kLLAlignDriveGains.kI,
//				LimelightConstants.kLLAlignDriveGains.kD);
//		DrivePIDController.setTolerance(0.5);
//
//		strafeOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignStrafeMotionSmoothing);
//		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignDriveMotionSmoothing);
//
//		// Use addRequirements() here to declare subsystem dependencies.
//		addRequirements(driveSubsystem);
//	}
//
//	// Called when the command is initially scheduled.
//	@Override
//	public void initialize() {
//
//
//
//			LimelightHelpers.setPipelineIndex("", LimelightConstants.kApriltagPipeline);
//
//
//			//LimelightHelpers.setPipelineIndex("", LimelightConstants.kReflectivePipeline);
//
//
//
//		LimelightHelpers.setLEDMode_ForceOff("");
//		Limelight.getInstance().setForTargeting(true);
//	}
//
//	// Called every time the scheduler runs while the command is scheduled.
//	@Override
//	public void execute() {
//
//		if (LimelightHelpers.getTV("")) {
//
//			SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));
//
//			double strafePIDOutput = StrafePIDController.calculate(LimelightHelpers.getTX(""), -.26);
//			//double drivePIDOutput = DrivePIDController.calculate(LimelightHelpers.getTY(""), .06);
//
//			double strafeOutput = strafeOutputSmoother.smoothInput(strafePIDOutput);
//			//double driveOutput = driveOutputSmoother.smoothInput(drivePIDOutput);
//
//			//in last year's code, we had the x and y values as seperate parameters, replaced by translation2D parameter
//			driveSubsystem.drive(new Translation2d(0, strafeOutput), 0,true, true);
//
//		} else {
//			driveSubsystem.drive(new Translation2d(0, 0), 0.0, true, true);
//		}
//	}
//
//	// Called once the command ends or is interrupted.
//	@Override
//	public void end(boolean interrupted) {
//		// LimelightHelpers.setLEDMode_PipelineControl("");
//	}
//
//	// Returns true when the command should end.
//	@Override
//	public boolean isFinished() {
//		return DrivePIDController.atSetpoint() && StrafePIDController.atSetpoint();
//	}
//}
