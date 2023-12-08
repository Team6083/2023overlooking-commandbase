// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand.ArmPIDControlCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSystem.LineSubsystem;

public class LinePIDControlCmd extends CommandBase {
  /** Creates a new LinePIDControlCmd. */
  final LineSubsystem lineSubsystem;
  public LinePIDControlCmd(LineSubsystem lineSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lineSubsystem = lineSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LineSubsystem.setPIDSetpoint(LineSubsystem.getPIDSetpoint());
    LineSubsystem.pidControlLoop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
