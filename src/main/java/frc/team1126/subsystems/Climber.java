package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swervelib.imu.SwerveIMU;
import frc.team1126.Constants.ClimberConstants;
import frc.team1126.Constants.ShooterConstants;

public class Climber extends SubsystemBase {
 
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
   
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private SwerveIMU imu;

    	private DigitalInput leftHome;
        private DigitalInput rightHome;


    private static final double MAX_HEIGHT = 100.0; // Define your max height here


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
    m_rightEncoder.setPosition(0);

}

    @Override
    public void periodic() {
        double leftHeight = m_leftEncoder.getPosition();
        double rightHeight = m_rightEncoder.getPosition();
        double averageHeight = (leftHeight + rightHeight) / 2.0;

        SmartDashboard.putNumber("Climber Height", averageHeight);
        SmartDashboard.putNumber("Roll", imu.getRoll());
    }

    public Command moveClimber(double leftY, double rightY) {

        return this.run(() -> setPower(leftY,rightY));
    }

    public Command moveClimberWithIMU(double leftY, double rightY) {

        return this.run(() -> moveMotorsWithIMU(leftY,rightY));
    }

    private void setPower(double leftY, double rightY) {
        if (leftHome.get() && leftY < 0) {
            motorLeft.set(0);
        } else {
            motorLeft.set(leftY);
        }

        if (rightHome.get() && rightY < 0) {
            motorRight.set(0);
        } else {
            motorRight.set(rightY);
        }
    }

    public void moveMotorsWithIMU(double leftY, double rightY) {
        double roll = imu.getRoll();

        if (roll != 0) {
            // If the roll is not 0, adjust the power of the motors to correct the roll.
            // This is a simple example and might need to be adjusted based on your specific requirements.
            double adjustment = roll * 0.01; // You might need to adjust this factor

            leftY -= adjustment;
            rightY += adjustment;
        }

        // Move the motors
        setPower(leftY, rightY);
    }


    public void moveToMax(double speed) {
        // Start moving the motors at the specified speed
        double leftY = speed;
        double rightY = speed;

        // Continuously check the current height of the motors
        while (true) {
            double roll = imu.getRoll();

            if (roll != 0) {
                // If the roll is not 0, adjust the power of the motors to correct the roll.
                // This is a simple example and might need to be adjusted based on your specific requirements.
                double adjustment = roll * 0.01; // You might need to adjust this factor

                leftY -= adjustment;
                rightY += adjustment;
            }

            // Move the motors
            setPower(leftY, rightY);

            double leftHeight = m_leftEncoder.getPosition();
            double rightHeight = m_rightEncoder.getPosition();

            // If the current height is equal to or greater than the max height, stop the motors
            if (leftHeight >= MAX_HEIGHT || rightHeight >= MAX_HEIGHT) {
                motorLeft.set(0);
                motorRight.set(0);
                break;
            }
        }
    }
    public void moveToHome(double speed) {
        // Start moving the motors in the negative direction at the specified speed
        double leftY = -speed;
        double rightY = -speed;

        // Continuously check the state of leftHome and rightHome
        while (true) {
            // If leftHome is true, stop the left motor
            if (leftHome.get()) {
                motorLeft.set(0);
                leftY = 0;
            } else {
                motorLeft.set(leftY);
            }

            // If rightHome is true, stop the right motor
            if (rightHome.get()) {
                motorRight.set(0);
                rightY = 0;
            } else {
                motorRight.set(rightY);
            }

            // If both leftHome and rightHome are true, break the loop
            if (leftHome.get() && rightHome.get()) {
                break;
            }
        }
    }

}
