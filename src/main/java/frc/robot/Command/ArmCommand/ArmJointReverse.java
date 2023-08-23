// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;

public class ArmJointReverse extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private boolean isReversed;

  /** Creates a new ArmJointReverse. */
  public ArmJointReverse(ArmSubsystem m_ArmSubsystem) {
    this.armSubsystem = m_ArmSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isReversed = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setJointReverse(isReversed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isReversed = false;
    armSubsystem.setJointReverse(isReversed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
