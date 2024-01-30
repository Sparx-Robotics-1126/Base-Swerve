package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.team1126.Constants.ShooterConstants;

public class Shooter {

    private CANSparkMax shooterMotor;
    private RelativeEncoder shooterEncoder;

    public Shooter() {
        shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ID,CANSparkLowLevel.MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
    }

}
