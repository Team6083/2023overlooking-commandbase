// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.ArmSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JointSubConstants;

public class JointSubsystem extends SubsystemBase {
  // motors
  private CANSparkMax armMotorLeft;
  private CANSparkMax armMotorRight;
  private MotorControllerGroup armMotor;

  // encoders
  private RelativeEncoder sparkMaxEncoder;
  private Encoder revEncoder;
  private double angleDegreeOffset;

  // pid controller
  private PIDController armPID;

  /** Creates a new JointSubsystem. */
  public JointSubsystem() {
    // motor
    armMotorLeft = new CANSparkMax(JointSubConstants.jointLeftCANId, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(JointSubConstants.jointRightCANId, MotorType.kBrushless);
    armMotor = new MotorControllerGroup(armMotorLeft, armMotorRight);
    armMotorLeft.setInverted(true);
    // encoder
    sparkMaxEncoder = armMotorLeft.getEncoder();
    revEncoder = new Encoder(JointSubConstants.revEncoderChannel1, JointSubConstants.revEncoderChannel2);
    angleDegreeOffset = JointSubConstants.jointInitAngleDegree;

    armPID = new PIDController(JointSubConstants.kP, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armInManualControlLoop(double manualSpeed) {
    armMotor.set(manualSpeed);
    armPID.setSetpoint(getAngleDegree());
  }

  public void pidControlLoop() {
    var armVolt = armPID.calculate(getAngleDegree());

    double modifiedArmVolt = armVolt;
    if (Math.abs(modifiedArmVolt) > JointSubConstants.jointVoltLimit) {
      modifiedArmVolt = JointSubConstants.jointVoltLimit * (armVolt > 0 ? 1 : -1);
    }
    armMotor.setVoltage(modifiedArmVolt);

    SmartDashboard.putNumber("arm_volt", modifiedArmVolt);
  }

  // PID get setpoint
  public double getSetpoint() {
    return armPID.getSetpoint();
  }

  // PID set setpoint
  public void setSetpoint(double setpoint) {
    final var currentSetpoint = getSetpoint();
    if (isPhyLimitExceed(currentSetpoint) != 0) {
      // if current setpoint exceed physical limit, don't do anything.
      return;
    }

    if (isPhyLimitExceed(setpoint) == -1) {
      setpoint = JointSubConstants.jointAngleMin;
    } else if (isPhyLimitExceed(setpoint) == 1) {
      setpoint = JointSubConstants.jointAngleMax;
    }

    armPID.setSetpoint(setpoint);
  }

  // encoder get angle
  public double getAngleDegree() {
    return getSparkMaxAngleDegree(); // change between getSparkMaxAngleDegree and getRevEncoderAngleDegree.
                                     // Depend on which encoder you use.
  }

  private double getSparkMaxAngleDegree() {
    SmartDashboard.putNumber("jointEncoderPos", sparkMaxEncoder.getPosition());
    return (sparkMaxEncoder.getPosition() * 360 / JointSubConstants.jointEncoderGearing) + angleDegreeOffset;
  }

  private double getRevEncoderAngleDegree() {
    return (revEncoder.get() * 360 / JointSubConstants.jointEncoderPulse) + angleDegreeOffset;
  }

  // reset encoder
  public void resetEncoder() {
    angleDegreeOffset = 0;
    resetSparkMaxEncoder();
    resetRevEncoder();
  }

  private void resetSparkMaxEncoder() {
    sparkMaxEncoder.setPosition(0);
  }

  private void resetRevEncoder() {
    revEncoder.reset();
  }

  public void resetSetpoint() {
    armPID.setSetpoint(0);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < JointSubConstants.jointAngleMin ? -1 : (angle > JointSubConstants.jointAngleMax ? 1 : 0));
  }

  public void pudDashboard() {
    SmartDashboard.putData("arm_PID", armPID);
    SmartDashboard.putData("arm_motor", armMotor);
  }
}
