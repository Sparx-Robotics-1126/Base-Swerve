// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team1126.commands.drive.DriveFieldRelative;
import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.SwerveSubsystem;
public class RobotContainer 
{

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  CommandXboxController driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
  CommandXboxController operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

  /* Operator Buttons */
  // private final JoystickButton cubeMode = new JoystickButton(operator.getHID(), XboxController.Button.kStart.value);
  // private final JoystickButton coneMode = new JoystickButton(operator.getHID(), XboxController.Button.kBack.value);

  public static final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  	// public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

  public RobotContainer() 
  {
    configureBindings();
    swerve.setDefaultCommand(new DriveFieldRelative(swerve, 
                                                    () -> driver.getRawAxis(translationAxis),
                                                    () -> driver.getRawAxis(strafeAxis),
                                                    () -> driver.getRawAxis(rotationAxis))); 
  }

  private void configureBindings() 
  {
    driver.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    driver.povUp().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2)));
    driver.povDown().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 - Math.PI)));
    driver.povLeft().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 - Math.PI/2)));
    driver.povRight().onTrue(new InstantCommand(() -> swerve.setHeadingAngle(Math.round(swerve.getYaw().getRadians() / (2.0*Math.PI)) * Math.PI * 2 + Math.PI/2)));
    driver.leftBumper().onTrue(new InstantCommand(() -> swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_SLOW_TRANSLATION)));
    driver.leftBumper().onFalse(new InstantCommand(() -> swerve.setTranslationalScalar(Constants.SwerveConstants.SWERVE_NORMAL_TRANSLATION)));
    // 	operator.start().onTrue(new InstantCommand(() ->m_candleSubsystem.setLEDSTate(CANdleSubsystem.LEDState.CONE)));
		// operator.back().onTrue(new InstantCommand(() -> m_candleSubsystem.setLEDSTate(CANdleSubsystem.LEDState.CUBE)));
  }
}
