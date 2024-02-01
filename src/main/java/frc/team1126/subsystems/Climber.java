package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swervelib.imu.SwerveIMU;
import frc.team1126.Constants.ClimberConstants;


public class Climber extends SubsystemBase {
 
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
   
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private SwerveIMU imu;

public Climber(SwerveSubsystem swerveSubsystem) {
    
    imu= swerveSubsystem.getSwerveIMU();
    motorLeft = new CANSparkMax(ClimberConstants.MOTOR_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    motorRight = new CANSparkMax(ClimberConstants.MOTOR_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);

    motorLeft.setInverted(true);
    m_leftEncoder = motorLeft.getEncoder();
    m_rightEncoder = motorRight.getEncoder();

    m_rightEncoder.setPositionConversionFactor(ClimberConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setPositionConversionFactor(ClimberConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder
        .setVelocityConversionFactor(Math.PI * ClimberConstants.kWheelDiameterMeters / ClimberConstants.kGearRatio / 60.0);
    m_rightEncoder
        .setVelocityConversionFactor(Math.PI * ClimberConstants.kWheelDiameterMeters / ClimberConstants.kGearRatio / 60.0);

    m_leftEncoder.setPosition(0);
   //Pigeon2Swerve.getInstance(); NEED TO BE PIGEON AWARE AT SOME POINT!

}

    @Override
    public void periodic() {
        double leftHeight = m_leftEncoder.getPosition();
        double rightHeight = m_rightEncoder.getPosition();
        double averageHeight = (leftHeight + rightHeight) / 2.0;

        SmartDashboard.putNumber("Climber Height", averageHeight);
        SmartDashboard.putNumber("Roll", imu.getRoll());
    }
}
