package frc.lib.Toolbox;

public class SwerveModuleConstants {

	public final int kCANCoderID;
	public final int kTurnMotorID;
	public final int kDriveMotorID;
	public final double kAngleZeroOffset;

	public SwerveModuleConstants(
			int CANCoderID,
			int TurnMotorID,
			int DriveMotorID,
			double AngleZeroOffset) {
		kCANCoderID = CANCoderID;
		kTurnMotorID = TurnMotorID;
		kDriveMotorID = DriveMotorID;
		kAngleZeroOffset = AngleZeroOffset;
	}

}