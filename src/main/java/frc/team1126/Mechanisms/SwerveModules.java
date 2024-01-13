// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126.Mechanisms;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderStatusFrame;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Toolbox.SparkMaxMaker;
import frc.lib.Toolbox.PIDGains;
import frc.lib.Toolbox.SwerveConstants;
import frc.lib.Toolbox.SwerveModuleConstants;
// import frc.robot.Constants;
import frc.robot.utils.AngleUtils;

public class SwerveModules {

	private final CANSparkMax turnMotor;
	private final CANSparkMax driveMotor;

	private final CTRECanCoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;
	private final AbsoluteEncoder throughBore;

	private final SparkMaxPIDController drivePID;
	private final SparkMaxPIDController turnPID;

	private final double angleZeroOffset;

	private final String moduleName;

	private SwerveModuleState optimizedState;

	private final SwerveConstants swerveConstants;

	/** Creates a new SwerveModule. */
	public SwerveModules(
			String moduleName,
			SwerveModuleConstants moduleConstants,
			SwerveConstants swerveConstants,
			boolean driveMotorReversed,
			boolean turningMotorReversed,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.swerveConstants = swerveConstants;
		this.moduleName = moduleName;
		this.angleZeroOffset = moduleConstants.kAngleZeroOffset;
		optimizedState = new SwerveModuleState();

//		// Initalize CANcoder
//		absoluteEncoder = new CTRECanCoder(moduleConstants.kCANCoderID);
//		absoluteEncoder.getConfigurator().apply(AngleUtils.generateCanCoderConfig());
//		absoluteEncoder.clearStickyFaults();

		// Initalize CANcoder
		absoluteEncoder = new CTRECanCoder(moduleConstants.kCANCoderID);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configAllSettings(AngleUtils.generateCanCoderConfig());
		// absoluteEncoder.configFactoryDefault();
		// absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		// // absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		// absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		// absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();


		// Initialize the motors
		// Initialize turning motor, encoder, and PID
		// Turn Motor
		turnMotor = SparkMaxMaker.createSparkMax(moduleConstants.kTurnMotorID);
		turnMotor.setInverted(turningMotorReversed);

		turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
		turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
		turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
		turnMotor.setIdleMode(IdleMode.kCoast);
		turnMotor.setSmartCurrentLimit(swerveConstants.kTurnSmartCurrentLimit);
		// Turn Encoder
		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio); // radians
		// radians per second
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio * (1d / 60d));
		// Turn PID
		turnPID = turnMotor.getPIDController();
		turnPID.setP(angularPIDGains.kP);
		turnPID.setI(angularPIDGains.kI);
		turnPID.setD(angularPIDGains.kD);
		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);

		// Leader Drive Motor
		driveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kDriveMotorID);
		driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
		driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
		driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
		// Set neutral mode to brake
		// leaderDriveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted(driveMotorReversed);
		driveMotor.setSmartCurrentLimit(swerveConstants.kDriveSmartCurrentLimit);
		// Leader Drive Encoder
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(
				swerveConstants.kDriveGearRatio * swerveConstants.kWheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				swerveConstants.kDriveGearRatio
						* swerveConstants.kWheelCircumference
						* (1d / 60d)); // meters per second

		// Leader Drive PID
		drivePID = driveMotor.getPIDController();
		drivePID.setP(drivePIDGains.kP);
		drivePID.setI(drivePIDGains.kI);
		drivePID.setD(drivePIDGains.kD);
		drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		drivePID.setSmartMotionMaxAccel(swerveConstants.kMaxModuleAccelMetersPerSecond, 0);
		drivePID.setSmartMotionMaxVelocity(swerveConstants.kMaxModuleSpeedMetersPerSecond, 0);
		drivePID.setFF(swerveConstants.kDriveFeedForward);

		// Follower Drive Motor
		// followerDriveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kFolloweDriveMotorID);
		// followerDriveMotor.follow(leaderDriveMotor, true);
		// followerDriveMotor.setSmartCurrentLimit(35);

		throughBore = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
		throughBore.setPositionConversionFactor((2 * Math.PI));
		throughBore.setZeroOffset(-angleZeroOffset);

		// turnPID.setFeedbackDevice(throughBore);
//		turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition().getValue() - angleZeroOffset));
		turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZeroOffset));

	}

	// Returns headings of the module

	/** returns the current heading of the module in radians */
	public double getHeading() {
		return turnEncoder.getPosition(); // % Math.PI;
	}

	/**
	 * returns the current heading of the module from the absolute encoder in
	 * radians
	 */
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition() - angleZeroOffset;
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	/**
	 * returns the error from the current angle to optimized angle of the module in
	 * radians
	 */
	public double getAngleError() {
		return optimizedState.angle.getRadians() - getHeading();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double moduleAngleRadians = getHeading();

		double distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}

	// region: Setters
	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, false);
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isTurbo) {

		double moduleAngleRadians = getHeading();

		// Optimize the reference state to avoid spinning further than 90 degrees to the
		// desired state
		optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		if (optimizedState.speedMetersPerSecond <= 0) {
			drivePID.setIAccum(0);
		}

		if (isTurbo) {
			// Squeeze every last bit if power out of turbo
			// leaderDriveMotor.setVoltage(12 *
			// Math.signum(optimizedState.speedMetersPerSecond));
			double turboSpeed = Math.copySign(
					swerveConstants.kMaxModuleSpeedMetersPerSecond,
					optimizedState.speedMetersPerSecond);

			drivePID.setReference(turboSpeed, ControlType.kSmartVelocity);
		} else {
			drivePID.setReference(
					optimizedState.speedMetersPerSecond,
					ControlType.kSmartVelocity);
		}

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber(moduleName + " Angle Error", Units.radiansToDegrees(getAngleError()));
		// SmartDashboard.putNumber(moduleName + " ThroughBore Angle",
		// Units.radiansToDegrees(throughBore.getPosition()));
		SmartDashboard.putNumber(moduleName + " Offset", angleZeroOffset);
//		SmartDashboard.putString(moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
		SmartDashboard.putNumber(moduleName + " Drive Current Draw", driveMotor.getOutputCurrent());
		SmartDashboard.putNumber(moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(moduleName + " Target Velocity", optimizedState.speedMetersPerSecond);
		SmartDashboard.putNumber(moduleName + " Velocity", driveEncoder.getVelocity());
		SmartDashboard.putNumber(moduleName + " SparkEncoder Angle",
				Units.radiansToDegrees(getHeading()));
	}

	// endregion: setters

}
