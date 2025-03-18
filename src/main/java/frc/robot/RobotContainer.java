// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.DriveSwerve;
import frc.robot.commands.swerve.ZeroHeading;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.team3526.driveControl.CustomController;
import lib.team3526.driveControl.CustomController.CustomControllerType;

public class RobotContainer {

  private final CustomController m_driverControllerCustom = new CustomController(0, CustomControllerType.PS5);
  
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

  private final SwerveDrive m_swerveDrive;
  private final Gyro gyro;

  SendableChooser<Command> autonomousChooser;

  public RobotContainer() {
    gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, gyro);

    SmartDashboard.putData(new ZeroHeading(m_swerveDrive));

    SendableChooser<Command> autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autonomousChooser);
    this.autonomousChooser = autonomousChooser;

    configureBindings();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -this.m_driverControllerCustom.getLeftY(),
        () -> this.m_driverControllerCustom.getLeftX(),
        () -> this.m_driverControllerCustom.getRightX(),
        () -> !this.m_driverControllerCustom.topButton().getAsBoolean()
      )
    );

  }

  public Command getAutonomousCommand() {
    return this.autonomousChooser.getSelected();
  }
}
