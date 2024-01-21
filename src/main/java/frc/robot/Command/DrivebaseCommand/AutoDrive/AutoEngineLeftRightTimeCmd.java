// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.DrivebaseCommand.AutoDrive;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Command.IntakeCommand.IntakeOffCmd;
import frc.robot.Subsystem.DrivebaseSubsystem;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Constants.AutoEnginePathConstants;

public class AutoEngineLeftRightTimeCmd extends CommandBase {
  public final DrivebaseSubsystem drivebaseSubsystem;
  // public final Supplier<Trajectory> trajectory;
  public final IntakeSubsystem intakeSubsystem;
  public final Timer timer;

  /** Creates a new AutoEngineTime. */
  public AutoEngineLeftRightTimeCmd(DrivebaseSubsystem drivebaseSubsystem, Timer timer, IntakeSubsystem intakeSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.timer = timer;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(drivebaseSubsystem, intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= 4) {
      autoArmControl(2, 1);
      drivebaseSubsystem.tankControl(0, 0);
    } else if (timer.get() > 4 && timer.get() <= 7) {
      autoArmControl(2, 0);
    } else if (timer.get() > 7 && timer.get() <= 8.5) {
      autoArmControl(5, 0);
    } else if (timer.get() > 8.5 && timer.get() <= 9) {
      drivebaseSubsystem.tankControl(0, 0);
      intakeSubsystem.solOff();
    } else if (timer.get() > 9 && timer.get() <= 13.1) {
      autoArmControl(5, 9);
      drivebaseSubsystem.tankControl(
          -AutoEnginePathConstants.lefttWheelVoltage,
          -AutoEnginePathConstants.rightWheelVoltage);
    } else if (timer.get() > 13.1 && timer.get() < 14.5) {
      drivebaseSubsystem.tankControl(0, 0);
      autoArmControl(2, 1);
    } else {
      intakeSubsystem.solOn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.tankControl(0, 0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
