// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.IntakeOnCmd;
import frc.robot.Command.DrivebaseCommand.AcradeDriveManulCmd;
import frc.robot.Command.DrivebaseCommand.TankDriveManulCmd;
import frc.robot.Constants.XboxControllerPortConstants;
import frc.robot.Subsystem.CameraSubsystem;
import frc.robot.Subsystem.DrivebaseSubsystem;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.LightSubsystem;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;
import frc.robot.Subsystem.ArmSystem.JointSubsystem;
import frc.robot.Subsystem.ArmSystem.LineSubsystem;

public class RobotContainer {
  // joystick
  CommandXboxController mainController;
  CommandXboxController viceController;
  // subsystem
  DrivebaseSubsystem m_DrivebaseSubsystem;
  ArmSubsystem m_ArmSubsystem;
  JointSubsystem m_JointSubsystem;
  LineSubsystem m_LineSubsystem;
  CameraSubsystem m_CameraSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  LightSubsystem m_LightSubsystem;

  public RobotContainer() {
    mainController = new CommandXboxController(XboxControllerPortConstants.kmain);
    viceController = new CommandXboxController(XboxControllerPortConstants.kvice);
    m_DrivebaseSubsystem = new DrivebaseSubsystem();
    m_ArmSubsystem = new ArmSubsystem();
    m_JointSubsystem = new JointSubsystem();
    m_LineSubsystem = new LineSubsystem();
    m_CameraSubsystem = new CameraSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_LightSubsystem = new LightSubsystem();

    m_DrivebaseSubsystem.setDefaultCommand(new AcradeDriveManulCmd(m_DrivebaseSubsystem,
        () -> mainController.getLeftY(), () -> mainController.getRightX()));
    configureBindings();
  }

  private void configureBindings() {
    mainController.a().whileTrue(new IntakeOnCmd(m_IntakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
