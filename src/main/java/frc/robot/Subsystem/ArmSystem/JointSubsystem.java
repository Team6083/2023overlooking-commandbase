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
  private CANSparkMax jointMotorLeft;
  private CANSparkMax jointMotorRight;
  private MotorControllerGroup jointMotor;

  // encoders
  private RelativeEncoder sparkMaxEncoder;
  private Encoder revEncoder;
  private double angleDegreeOffset;

  // pid controller
  private PIDController jointPID;

  /** Creates a new JointSubsystem. */
  public JointSubsystem() {
    // motor
    jointMotorLeft = new CANSparkMax(JointSubConstants.jointLeftCANId, MotorType.kBrushless);
    jointMotorRight = new CANSparkMax(JointSubConstants.jointRightCANId, MotorType.kBrushless);
    jointMotor = new MotorControllerGroup(jointMotorLeft, jointMotorRight);
    jointMotorLeft.setInverted(true);
    // encoder
    sparkMaxEncoder = jointMotorLeft.getEncoder();
    revEncoder = new Encoder(JointSubConstants.revEncoderChannel1, JointSubConstants.revEncoderChannel2);
    angleDegreeOffset = JointSubConstants.jointInitAngleDegree;
    jointPID = new PIDController(JointSubConstants.kP, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDashboard();
  }

  public void armInManualControlLoop(double manualSpeed) {
    jointMotor.set(manualSpeed);
    jointPID.setSetpoint(getAngleDegree());
  }

  public void pidControlLoop() {
    var jointVolt = jointPID.calculate(getAngleDegree());

    double modifiedArmVolt = jointVolt;
    if (Math.abs(modifiedArmVolt) > JointSubConstants.jointVoltLimit) {
      modifiedArmVolt = JointSubConstants.jointVoltLimit * (jointVolt > 0 ? 1 : -1);
    }
    jointMotor.setVoltage(modifiedArmVolt);

    SmartDashboard.putNumber("joint_volt", modifiedArmVolt);
  }

  // PID get setpoint
  public double getSetpoint() {
    return jointPID.getSetpoint();
  }

  // PID set setpoint
  public void setSetpoint(double setpoint) {
    final var currentSetpoint = getSetpoint();
    if (isPhyLimitExceed(currentSetpoint) != 0) { // if current setpoint exceed physical limit, don't do anything.
      return;
    }

    if (isPhyLimitExceed(setpoint) == -1) {
      setpoint = JointSubConstants.jointAngleMin;
    } else if (isPhyLimitExceed(setpoint) == 1) {
      setpoint = JointSubConstants.jointAngleMax;
    }

    jointPID.setSetpoint(setpoint);
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
    jointPID.setSetpoint(0);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < JointSubConstants.jointAngleMin ? -1 : (angle > JointSubConstants.jointAngleMax ? 1 : 0));
  }

  public void putDashboard() {
    SmartDashboard.putData("joint_PID", jointPID);
    SmartDashboard.putData("joint_motor", jointMotor);
  }
}
