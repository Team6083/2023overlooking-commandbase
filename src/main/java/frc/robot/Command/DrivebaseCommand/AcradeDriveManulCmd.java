// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.DrivebaseCommand;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.DrivebaseSubsystem;

public class AcradeDriveManulCmd extends CommandBase {
  /** Creates a new AcradeDriveManulCmd. */
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final Supplier<Double> power, rotation;

  public AcradeDriveManulCmd(DrivebaseSubsystem m_DrivebaseSubsystem, Supplier<Double> power, Supplier<Double> rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DrivebaseSubsystem = m_DrivebaseSubsystem;
    this.power = power;
    this.rotation = rotation;
    addRequirements(m_DrivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivebaseSubsystem.arcadeControl(power.get(), rotation.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivebaseSubsystem.arcadeControl(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
