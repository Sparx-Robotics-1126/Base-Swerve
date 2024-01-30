package frc.team1126.commands.Limelight;

import java.util.function.DoubleSupplier;

import frc.team1126.subsystems.SwerveSubsystem;

public class DriveToDistance extends VisionAlignment{
    
  public DriveToDistance(DoubleSupplier forwardSpeed, double angle, SwerveSubsystem swerve) {
        super(forwardSpeed, angle, swerve);
    }

public void moveToDistance() {
	double tempVar = 0.0;
    
    if(limelight.getDistance() < tempVar) {
		super.execute();

	} else if(limelight.getDistance() > tempVar) {
		super.execute();
	} else {
        super.end(true);
	}
  }
  
}
