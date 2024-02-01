package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooterMotor;
    private RelativeEncoder shooterEncoder;

    private double shooterSpeed;

    public Shooter() {
        shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
    }

    public void setShooterSpeed(double speed) {
        shooterSpeed = speed;
        shooterMotor.set(speed);
    }

    public double getShooterSpeed() {
        return shooterMotor.get();
    }

    public boolean isMotorUpToSpeed() {
        double currentSpeed = shooterMotor.get();
        return currentSpeed >= shooterSpeed;
    }

}
