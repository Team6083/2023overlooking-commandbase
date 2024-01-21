// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.ArmCommand.ArmCatchCmd;
import frc.robot.Command.ArmCommand.ArmDoubleSustationCmd;
import frc.robot.Command.ArmCommand.ArmHighNodeCmd;
import frc.robot.Command.ArmCommand.ArmJointReverse;
import frc.robot.Command.ArmCommand.ArmMiddleNodeCmd;
import frc.robot.Command.ArmCommand.ArmVerticalCmd;
import frc.robot.Command.ArmCommand.ArmManualCommand.JointManulCmd;
import frc.robot.Command.ArmCommand.ArmPIDControlCommand.JointPIDControlCmd;
import frc.robot.Command.DrivebaseCommand.AcradeDriveManulCmd;
import frc.robot.Command.DrivebaseCommand.TankDriveManulCmd;
import frc.robot.Command.IntakeCommand.CompreOnCmd;
import frc.robot.Command.IntakeCommand.IntakeOnCmd;
import frc.robot.Constants.XboxControllerPortConstants;
import frc.robot.Constants.JointSubConstants;
import frc.robot.Constants.LineSubConstants;
import frc.robot.Subsystem.CameraSubsystem;
import frc.robot.Subsystem.DrivebaseSubsystem;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.LightSubsystem;
import frc.robot.Subsystem.ArmSystem.ArmSubsystem;
import frc.robot.Subsystem.ArmSystem.JointSubsystem;
import frc.robot.Subsystem.ArmSystem.LineSubsystem;

public class RobotContainer {
  // joystick
  CommandXboxController mainController;
  CommandXboxController viceController;
  // subsystem
  DrivebaseSubsystem drivebaseSubsystem;
  ArmSubsystem armSubsystem;
  JointSubsystem jointSubsystem;
  LineSubsystem lineSubsystem;
  CameraSubsystem cameraSubsystem;
  IntakeSubsystem intakeSubsystem;
  LightSubsystem lightSubsystem;
  // Asis
  double mainLeftTriggerValue;
  double mainRightTrigggerValue;
  SendableChooser<Command> chooser;

  public RobotContainer() {
    // xboxController
    mainController = new CommandXboxController(XboxControllerPortConstants.kmain);
    viceController = new CommandXboxController(XboxControllerPortConstants.kvice);
    // subsystem
    drivebaseSubsystem = new DrivebaseSubsystem();
    armSubsystem = new ArmSubsystem();
    jointSubsystem = new JointSubsystem();
    lineSubsystem = new LineSubsystem();
    cameraSubsystem = new CameraSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    lightSubsystem = new LightSubsystem();

    // axis
    mainLeftTriggerValue = mainController.getLeftTriggerAxis();
    mainRightTrigggerValue = mainController.getRightTriggerAxis();

    drivebaseSubsystem.setDefaultCommand(new AcradeDriveManulCmd(drivebaseSubsystem,
        () -> mainController.getLeftY(), () -> mainController.getRightX()));

    intakeSubsystem.setDefaultCommand(new SequentialCommandGroup(new IntakeOnCmd(intakeSubsystem), new CompreOnCmd(intakeSubsystem)));

    configureBindings();
    chooser = new SendableChooser<>();
    
  }

  private void configureBindings() {
    // intake
    mainController.y().onTrue(new IntakeOnCmd(intakeSubsystem));
   

    // arm
    mainController.a().whileFalse(new JointPIDControlCmd(armSubsystem, mainLeftTriggerValue, mainRightTrigggerValue));
    mainController.a().whileTrue(new JointManulCmd(armSubsystem, mainLeftTriggerValue, mainRightTrigggerValue));
    viceController.back().whileTrue(new ArmJointReverse(armSubsystem));
    mainController.pov(90).onTrue(new ArmVerticalCmd(armSubsystem, JointSubConstants.jointVerticalSetpoints,
        LineSubConstants.lineVerticalSetpoints));
    viceController.leftBumper()
        .onTrue(new ArmMiddleNodeCmd(armSubsystem, JointSubConstants.jointMiddleNodeSetpoints,
            LineSubConstants.lineMiddleNodeSetpoints));
    viceController.rightBumper().onTrue(new ArmHighNodeCmd(armSubsystem, JointSubConstants.jointHighNodeSetpoints,
        LineSubConstants.lineHighNodeSetpoints));
    viceController.pov(90).onTrue(new ArmCatchCmd(armSubsystem, JointSubConstants.jointCatchSetpoints,
        LineSubConstants.lineCatchSetpoints));
    viceController.b()
        .onTrue(new ArmDoubleSustationCmd(armSubsystem, JointSubConstants.jointDoubleSubstationSetpoints,
            LineSubConstants.lineDoubleSubstationSetpoints));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
