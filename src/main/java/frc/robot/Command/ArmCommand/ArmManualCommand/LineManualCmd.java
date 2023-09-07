// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ArmCommand.ArmManualCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.ArmSystem.LineSubsystem;

public class LineManualCmd extends CommandBase {
  private final LineSubsystem lineSubsystem;
  private double POVnumericalValue;
  private final CommandXboxController main;
  /** Creates a new LineManualCmd. */
  public LineManualCmd(LineSubsystem line_Subsystem,double POVnumericalValue, CommandXboxController main) {
    this.lineSubsystem = line_Subsystem;
    this.POVnumericalValue = POVnumericalValue;
    this.main = main;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lineSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double linemainController = POVnumericalValue;
    lineSubsystem.SpeedbottonControlLoop(linemainController);
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
