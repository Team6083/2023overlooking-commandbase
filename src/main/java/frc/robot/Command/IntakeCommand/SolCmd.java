// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.IntakeCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.IntakeSubsystem;

public class SolCmd extends CommandBase {
  /** Creates a new IntakeCmd. */
  private final IntakeSubsystem IntakeSubsystem;

  public SolCmd(IntakeSubsystem IntakeSubsystem) {
    this.IntakeSubsystem = IntakeSubsystem;
    addRequirements(IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.solForward = ! IntakeSubsystem.solForward;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.sol(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
