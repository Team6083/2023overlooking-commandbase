// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.ArmSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.ArmSystem.JointSubsystem;
import frc.robot.Constants.ArmSubConstants;

public class ArmSubsystem extends SubsystemBase {
  protected JointSubsystem joint;
  protected LineSubsystem line;
  private int jointIsReversed;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    joint = new JointSubsystem();
    line = new LineSubsystem();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDashboard();
  }

  public void jointPIDControlLoop() {
    joint.pidControlLoop();
    // line.pidControlLoop();
  }

  public void linePIDControlLoop() {
    // line.pidControlLoop();
  }

  public double getAngleDegree() {
    return joint.getAngleDegree();
  }

  // public double getLength() {
  // return line.getLineLength();
  // }

  public void setAngleSetpoint(double angleSetPoint) {
    joint.setSetpoint(angleSetPoint);
    return;
  }

  // public void setLineSetpoint(double lineSetPoint) {
  // line.setPIDSetpoint(lineSetPoint);
  // return;
  // }

  public double getJointSetpoint() {
    return joint.getSetpoint();
  }

  public double getLineMaxLengthByJointAngle(double jointAngleRadian) {
    double angle = jointAngleRadian;

    double maxProj = ArmSubConstants.jointToFrameDist + ArmSubConstants.extendLimit;
    double maxLenByExtendLimit = (maxProj / Math.abs(Math.cos(angle))) - ArmSubConstants.firstStageToJoint;

    double maxLenByHeightLimit = ((ArmSubConstants.heightLimit - ArmSubConstants.jointHeight)
        / Math.abs(Math.sin(angle))) - ArmSubConstants.firstStageToJoint;

    return Math.min(maxLenByExtendLimit, maxLenByHeightLimit);
  }

  public void setJointReverse(boolean isReversed) {
    jointIsReversed = (isReversed ? 1 : 0);
  }

  public int getJointReverse() {
    return jointIsReversed;
  }

  public void resetJoint() {
    joint.resetEncoder();
    joint.resetSetpoint();
  }

  public void resetLine() {

  }

  public void putDashboard() {
    SmartDashboard.putNumber("joint_angle", getAngleDegree());
    // SmartDashboard.putNumber("line_length", getLength());
  }
}
