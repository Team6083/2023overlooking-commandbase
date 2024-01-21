// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseSubConstants;
import frc.robot.Robot;

public class DrivebaseSubsystem extends SubsystemBase {
  /** Creates a new DrivebaseSubsystem. */

  public WPI_TalonSRX leftMotor1;
  public WPI_VictorSPX leftMotor2;
  public WPI_TalonSRX rightMotor1;
  public WPI_VictorSPX rightMotor2;// Construct MotorController

  public MotorControllerGroup leftMotor;
  public MotorControllerGroup rightMotor;
  public DifferentialDrive drive;// Used to simplify drivebase program

  // Sensor
  public AHRS gyro;

  public DifferentialDriveOdometry odometry;
  protected DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      DrivebaseSubConstants.kTrackWidthMeters);
  protected RamseteController ramseteController = new RamseteController();

  // Feedforward Controller
  protected SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      DrivebaseSubConstants.ks, DrivebaseSubConstants.kv);

  protected Field2d field = new Field2d();
  protected Field2d trajField = new Field2d();

  public PIDController leftPID = new PIDController(DrivebaseSubConstants.kP, DrivebaseSubConstants.kI,
      DrivebaseSubConstants.kD);
  public PIDController rightPID = new PIDController(DrivebaseSubConstants.kP, DrivebaseSubConstants.kI,
      DrivebaseSubConstants.kD);

  private double outputRatio;

  private double leftMotorVolt;
  private double rightMotorVolt;

  private double leftMotorSpeedInput;
  private double rightMotorSpeedInput;

  private double goalLeftWheelSpeed;
  private double goalRightWheelSpeed;

  private double leftPos;
  private double rightPos;

  public DrivebaseSubsystem() {
    leftMotor1 = new WPI_TalonSRX(DrivebaseSubConstants.leftMotorID1);
    leftMotor2 = new WPI_VictorSPX(DrivebaseSubConstants.leftMotorID2);
    rightMotor1 = new WPI_TalonSRX(DrivebaseSubConstants.rightMotorID1);
    rightMotor2 = new WPI_VictorSPX(DrivebaseSubConstants.rightMotorID2);

    leftMotor = new MotorControllerGroup(leftMotor1, leftMotor2);
    rightMotor = new MotorControllerGroup(rightMotor1, rightMotor2);
    leftMotor.setInverted(true);
    leftMotor1.setSensorPhase(true);
    drive = new DifferentialDrive(leftMotor, rightMotor);

    // Reset encoder
    resetEncoder();

    // Define gyro ID
    gyro = new AHRS(SPI.Port.kMXP);

    // Put path and status on field from PathWeaver on SmartDashboard
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0),
        positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
        positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("field", field);
    SmartDashboard.putData("trajField", trajField);

    putDashboard();
  }

  public void setOutputRadio(double radio) {
    outputRatio = radio;
  }

  public double getOutputRatio() {
    return outputRatio;
  }

  public void tankControl(double leftMotorInput, double rightMotorInput) {
    drive.tankDrive(leftMotorInput, rightMotorInput);
  }

  public void arcadeControl(double MotorPowerInput, double RotationInput) {
    drive.arcadeDrive(MotorPowerInput, RotationInput);
  }

  public void runTraj(Trajectory trajectory, double timeInsec) {

    // Set the goal of the robot in that second
    Trajectory.State goal = trajectory.sample(timeInsec);
    trajField.setRobotPose(goal.poseMeters);

    var currentPose = odometry.getPoseMeters();

    var goalChaspeed = ramseteController.calculate(currentPose, goal);

    // Convert chassis speed to wheel speed
    var goalWheelSpeeds = kinematics.toWheelSpeeds(goalChaspeed); // Left and right speed
    goalLeftWheelSpeed = goalWheelSpeeds.leftMetersPerSecond; // Catch speed from wheelSpeed(with ctrl + left mice)
    goalRightWheelSpeed = goalWheelSpeeds.rightMetersPerSecond;

    double currLeftSpeed = positionToDistanceMeter(leftMotor1.getSelectedSensorVelocity());
    double currRightSpeed = positionToDistanceMeter(rightMotor1.getSelectedSensorVelocity());

    leftPID.setSetpoint(goalLeftWheelSpeed);
    rightPID.setSetpoint(goalRightWheelSpeed);

    leftMotorVolt = leftPID.calculate(currLeftSpeed) + feedforward.calculate(goalLeftWheelSpeed);
    rightMotorVolt = rightPID.calculate(currRightSpeed) + feedforward.calculate(goalRightWheelSpeed);

    leftMotor.setVoltage(leftMotorVolt);
    rightMotor.setVoltage(rightMotorVolt);
    drive.feed();

    // put the distances between the target and current position onto Dashboard
    SmartDashboard.putNumber("left_volt", leftMotorVolt);
    SmartDashboard.putNumber("right_volt", rightMotorVolt);

    SmartDashboard.putNumber("errorPosX", currentPose.minus(goal.poseMeters).getX());
    SmartDashboard.putNumber("errorPosY", currentPose.minus(goal.poseMeters).getY());

  }

  public void updateODO() {
    var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
    leftPos = positionToDistanceMeter(leftMotor1.getSelectedSensorPosition());
    rightPos = positionToDistanceMeter(rightMotor1.getSelectedSensorPosition());

    odometry.update(gyroAngle, leftPos, rightPos);
    field.setRobotPose(odometry.getPoseMeters());

    /*
     * // number on Dashboard can be adjusted
     * kP = SmartDashboard.getNumber("kP", kP);
     * kI = SmartDashboard.getNumber("kI", kI);
     * kD = SmartDashboard.getNumber("kD", kD);
     * 
     * leftPID.setPID(kP, kI, kD);
     * rightPID.setPID(kP, kI, kD);
     */

    // drive.feed();
  }

  // To set the position and rotation of the robot
  public void setODOPose(Pose2d pose) {
    Rotation2d rotation = pose.getRotation();
    leftPos = positionToDistanceMeter(leftMotor1.getSelectedSensorPosition());
    rightPos = positionToDistanceMeter(rightMotor1.getSelectedSensorPosition());

    odometry.resetPosition(rotation, leftPos, rightPos, pose);
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPID() {
    leftPID.reset();
    rightPID.reset();
  }

  public double getGyroDegree() {
    return gyro.getPitch();
  }

  // Calculate the distance(meter)
  public double positionToDistanceMeter(double position) {
    double sensorRate = position / DrivebaseSubConstants.encoderPulse;
    double wheelRate = sensorRate * 0.5;

    double positionMeter = 2 * Math.PI * Units.inchesToMeters(6) * wheelRate;
    return positionMeter;
  }

  // Here comes some mode to set up or update
  public void resetEncoder() {
    leftMotor1.setSelectedSensorPosition(0);
    rightMotor1.setSelectedSensorPosition(0);
  }

  // Input the position of the encoder
  public void putDashboard() {
    SmartDashboard.putNumber("leftEncoder", leftMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightEncoder", rightMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    SmartDashboard.putNumber("leftController_speed", leftMotorSpeedInput);
    SmartDashboard.putNumber("rightController_speed", rightMotorSpeedInput);
    SmartDashboard.putNumber("goal_left_wheel_speed", goalLeftWheelSpeed);
    SmartDashboard.putNumber("goal_right_wheel_speed", goalRightWheelSpeed);
    SmartDashboard.putNumber("left_pos", leftPos);
    SmartDashboard.putNumber("right_pos", rightPos);

    SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());
  }
}
