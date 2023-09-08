// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public Compressor com;
  public DoubleSolenoid sol;
  public boolean solForward = false;
  public boolean compre = true;

  public IntakeSubsystem() {
    com = new Compressor(PneumaticsModuleType.CTREPCM);
    sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    com.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void sol(boolean on) {
    if (on) {
      sol.set(Value.kForward);
    } else {
      sol.set(Value.kOff);
    }
  }
}
