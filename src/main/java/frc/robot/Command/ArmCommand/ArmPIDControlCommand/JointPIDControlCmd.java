// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand.ArmPIDControlCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;

public class JointPIDControlCmd extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private double leftTriggerValue;
  private double rightTriggerValue;
  private double armAngleModify;

  /** Creates a new ArmPIDControlCmd. */
  public JointPIDControlCmd(ArmSubsystem m_ArmSubsystem, double m_m_leftTrigger, double m_m_rightTrigger) {
    this.armSubsystem = m_ArmSubsystem;
    this.leftTriggerValue = m_m_leftTrigger;
    this.rightTriggerValue = m_m_rightTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngleModify = (leftTriggerValue - rightTriggerValue) * -0.7;
    armSubsystem.setAngleSetpoint(armSubsystem.getJointSetpoint() + armAngleModify);
    armSubsystem.jointPIDControlLoop();
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
