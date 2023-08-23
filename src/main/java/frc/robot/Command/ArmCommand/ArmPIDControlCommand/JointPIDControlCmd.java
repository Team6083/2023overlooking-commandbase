// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand.ArmPIDControlCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;

public class JointPIDControlCmd extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private ArmSubsystem arm;

  /** Creates a new ArmPIDControlCmd. */
  public JointPIDControlCmd(ArmSubsystem m_ArmSubsystem) {
    this.armSubsystem = m_ArmSubsystem;
    arm = new ArmSubsystem();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setJointReverse(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.jointPIDControlLoop();
  }

  // Called once the command ends or is
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
