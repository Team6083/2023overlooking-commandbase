// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.AutoModeCmdGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Command.ArmCommand.ArmCatchCmd;
import frc.robot.Command.DrivebaseCommand.AutoDrive.AutoEngineMiddleTimeCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoEngineMiddleUp extends SequentialCommandGroup {
  /** Creates a new AutoEngineBackward. */
  public AutoEngineMiddleUp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoEngineMiddleTimeCmd(null, null, null),
    new ArmCatchCmd(null, null, null), new AutoEngineMiddleTimeCmd(null, null, null));
    // write the real middle timer
    // need to include middletime cmd and doMiddle up
  }
}
