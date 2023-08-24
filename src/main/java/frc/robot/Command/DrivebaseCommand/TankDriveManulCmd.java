// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.DrivebaseCommand;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.DrivebaseSubConstants;
import frc.robot.Subsystem.DrivebaseSubsystem;

public class TankDriveManulCmd extends CommandBase {
  public final DrivebaseSubsystem m_drivebaseSubsystem;
  public final Supplier<Double> LMI, RMI;
  // public final double RMI;
  /** Creates a new TankDriveManulCmd. */
  public TankDriveManulCmd(DrivebaseSubsystem drivebaseSubsystem, Supplier<Double> LMI, Supplier<Double> RMI) {
    this.m_drivebaseSubsystem = drivebaseSubsystem;
    this.LMI = LMI;
    this.RMI = RMI;
    addRequirements(drivebaseSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivebaseSubsystem.putDashboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebaseSubsystem.tankControl(LMI.get(), RMI.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebaseSubsystem.tankControl(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
