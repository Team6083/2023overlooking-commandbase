// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.DrivebaseCommand.AutoDrive;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.DrivebaseSubConstants;
import frc.robot.Subsystem.DrivebaseSubsystem;

public class AutoArmControl extends CommandBase {
    public final DrivebaseSubsystem drivebaseSubsystem;
    // public final Arm
    // public final Supplier<Trajectory> trajectory;
    public final Timer timer;

    /** Creates a new AutoEnginePath. */
    public AutoArmControl(DrivebaseSubsystem drivebaseSubsystem, Timer timer) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        // this.trajectory = trajectory;
        this.timer = timer;
        addRequirements(drivebaseSubsystem);
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

        // drivebaseSubsystem.runTraj(trajectory.get(), timer.get());
    } // originally a function and have to get the variable?

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.tankControl(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
