// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;

public class ArmMiddleNodeCmd extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double[] jointSetpoint;
  private ArmSubsystem arm;

  /** Creates a new ArmMiddleNode. */
  public ArmMiddleNodeCmd(ArmSubsystem m_ArmSubsystem, double[] j_setPoint, double l_setPoint) {
    this.armSubsystem = m_ArmSubsystem;
    this.jointSetpoint = j_setPoint;
    arm = new ArmSubsystem();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setAngleSetPoint(jointSetpoint[arm.getJointReverse()]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
