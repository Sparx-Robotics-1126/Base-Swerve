package frc.team1126.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;

public class DriveToDistance extends Command {

        private static double yP = 0.1;
    private static double yI = 0.0;
    private static double yD = 0.0;
    private final SwerveSubsystem swerve;
    private final Limelight limelight;
    private final PIDController distanceController;
    private static byte visionPipeline =0;
    public DriveToDistance(SwerveSubsystem swerve,  double targetDistance) {
        this.swerve = swerve;
        this.limelight = new Limelight();
        this.distanceController = new PIDController(yP, yI, yD);
        this.distanceController.setSetpoint(targetDistance);
        addRequirements(swerve);
        setVisionPipeline(visionPipeline);
    }


    public static void setVisionPipeline(byte pipeline) {
        visionPipeline = pipeline;
    }
    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            double currentDistance = limelight.getDistance();
            double speed = distanceController.calculate(currentDistance);
            swerve.drive(new Translation2d(0.0, speed), 0.0, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return distanceController.atSetpoint();
    }
}
