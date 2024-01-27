// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.SwerveConstants;
//import frc.team1126.commands.Limelight.LLAlignCommand;
//import frc.team1126.commands.drive.DriveFieldRelative;
import frc.team1126.commands.drivebase.AbsoluteDriveAdv;
// import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;
import frc.team1126.Constants.OIConstants;
import frc.team1126.Constants.OperatorConstants;
public class RobotContainer 
{

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),   "swerve"));

  XboxController driverXbox = new XboxController(0);

  CommandXboxController driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
  CommandXboxController operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

  public RobotContainer() 
  {

    
    configureBindings();
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                    OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                    OperatorConstants.RIGHT_X_DEADBAND),
            driverXbox::getYButtonPressed,
            driverXbox::getAButtonPressed,
            driverXbox::getXButtonPressed,
            driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverXbox.getRightX(),
            () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
            !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

                  
  }

  private void configureBindings() 
  {
//    driver.leftTrigger().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
//    driver.x().onTrue(new LLAlignCommand(true));
//    driver.povUp().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2)));
//    driver.povDown().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 - Math.PI)));
//    driver.povLeft().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 - Math.PI/2)));
//    driver.povRight().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 + Math.PI/2)));
//    driver.leftBumper().onTrue(new InstantCommand(() -> swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_SLOW_TRANSLATION)));
//    driver.leftBumper().onFalse(new InstantCommand(() -> swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_NORMAL_TRANSLATION)));
    // operator.start().onTrue(new InstantCommand(() ->m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CONE)));
		// operator.back().onTrue(new InstantCommand(() -> m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CUBE)));
    operator.x().onTrue(new InstantCommand(() ->  operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)));
  }



  public void EndGameRumble() {

    if(DriverStation.getMatchTime() < SwerveConstants.EndGameSeconds && DriverStation.getMatchTime() > SwerveConstants.StopRumbleSeconds) {

      driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);

    } else {
      driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
  }

  /* Operator Buttons */
  // private final JoystickButton cubeMode = new JoystickButton(operator.getHID(), XboxController.Button.kStart.value);
  // private final JoystickButton coneMode = new JoystickButton(operator.getHID(), XboxController.Button.kBack.value);



  //  public final static SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  // public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();
  public final Limelight m_limeLight = new Limelight();
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
