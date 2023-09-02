// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.ArmSystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LineSubConstants;

public class LineSubsystem extends SubsystemBase {
  // motor
  private static WPI_TalonSRX lineMotor;

  //PID controller
  protected static PIDController linePID;
  private static double lineLengthOffset;

  /** Creates a new ExampleSubsystem. */
  public LineSubsystem() {
    lineMotor = new WPI_TalonSRX(LineSubConstants.lineId);
    lineMotor.setInverted(true);
    lineMotor.setSensorPhase(true);
    resetEncoder();

    linePID = new PIDController(LineSubConstants.kP, LineSubConstants.kI, LineSubConstants.kD);
    linePID.setSetpoint(LineSubConstants.lineInitLength);

    lineLengthOffset = LineSubConstants.lineInitLength;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDashboard();
  }

  public void manualControlLoop(double manualControlSpeed) {
    lineMotor.set(manualControlSpeed);
    linePID.setSetpoint(getLineLength());
}

public static void pidControlLoop() {
  double lineVolt = linePID.calculate(getLineLength());
  if (lineVolt > LineSubConstants.modifiedLineVoltPLimit) {
      lineVolt = LineSubConstants.modifiedLineVoltPLimit;
  } else if (lineVolt < LineSubConstants.modifiedLineVoltNLimit) {
      lineVolt = LineSubConstants.modifiedLineVoltNLimit;
  }
  lineMotor.setVoltage(lineVolt);

  SmartDashboard.putNumber("line_volt", lineVolt);
}

public static double getPIDSetpoint() {
  return linePID.getSetpoint();
}

public static void setPIDSetpoint(double setpoint) {
  final double currentSetpoint = linePID.getSetpoint();
  if (isPhyLimitExceed(currentSetpoint) != 0) {
      return;
  }
  if (isPhyLimitExceed(setpoint) == -1) {
      setpoint = LineSubConstants.minLineLengthLimit;
  } else if (isPhyLimitExceed(setpoint) == 1) {
      setpoint = LineSubConstants.maxLineLengthLimit;
  }
  linePID.setSetpoint(setpoint);
}

public void resetEncoder() {
  lineLengthOffset = 0;
  lineMotor.setSelectedSensorPosition(0);
}

public void resetSetpoint() {
  linePID.setSetpoint(40);
}

public void stopMotor() {
  lineMotor.stopMotor();
}

public static double getLineLength() {
  double x = lineMotor.getSelectedSensorPosition();
  double cal1 = 0.00473 * x;
  double cal2 = 0.0000000348 * x * x;
  double length = cal1 - cal2;
  return length + lineLengthOffset;
}

private static int isPhyLimitExceed(double angle) {
  return angle < LineSubConstants.minLineLengthLimit ? -1 : (angle > LineSubConstants.maxLineLengthLimit ? 1 : 0);
}

public void putDashboard() {
  SmartDashboard.putData("line_PID", linePID);
  SmartDashboard.putData("line_motor", lineMotor);
}
}
