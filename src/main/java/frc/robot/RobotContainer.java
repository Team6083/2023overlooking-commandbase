// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.DrivebaseCommand.AcradeDriveManulCmd;
import frc.robot.Command.DrivebaseCommand.TankDriveManulCmd;
import frc.robot.Constants.XboxControllerPortConstants;
import frc.robot.Subsystem.DrivebaseSubsystem;

public class RobotContainer {

  private final CommandXboxController mainController = new CommandXboxController(XboxControllerPortConstants.kmain);
  private final CommandXboxController viceController = new CommandXboxController(XboxControllerPortConstants.kvice);

  private final DrivebaseSubsystem m_DrivebaseSubsystem = new DrivebaseSubsystem();

  public RobotContainer() {
    configureBindings();

    m_DrivebaseSubsystem.setDefaultCommand(new AcradeDriveManulCmd(m_DrivebaseSubsystem,
        () -> mainController.getLeftY(), () -> mainController.getRightX()));

  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
