// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  // Left side of the drivetrain

  public CANSparkMax leftLeader1 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_1, MotorType.kBrushless);
  public CANSparkMax leftFollower2 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_2, MotorType.kBrushless);
  public CANSparkMax leftFollower3 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_3, MotorType.kBrushless);
  // Right side of the drivetrain
  private final CANSparkMax rightLeader1 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_1,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax rightFollower2 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_2,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax rightFollower3 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_3,
      CANSparkMax.MotorType.kBrushless);


  // Differential Drive
  private final DifferentialDrive driveTrain = new DifferentialDrive(leftLeader1, rightLeader1);
  private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(
      RobotContainer.navX.getRotation2d());
  private final SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
      Constants.Drive.Auton.ksVolts, Constants.Drive.Auton.kvVoltSecondsPerMeter,
      Constants.Drive.Auton.kaVoltSecondsSquaredPerMeter);
  private final PIDController m_leftPIDController = new PIDController(Constants.Drive.Auton.kPDriveVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(Constants.Drive.Auton.kPDriveVel, 0, 0);


  /** Creates a new DriveTrain. */
  
  public DriveTrain() {
    leftLeader1.setInverted(Constants.Drive.DriveMotors.LEFT_INVERTED);
    rightLeader1.setInverted(Constants.Drive.DriveMotors.RIGHT_INVERTED);
    leftFollower2.follow(leftLeader1);
    leftFollower3.follow(leftLeader1);
    rightFollower2.follow(rightLeader1);
    rightFollower3.follow(rightLeader1);
    setConversionFactor(leftLeader1);
    setConversionFactor(leftFollower2);
    setConversionFactor(leftFollower3);
    setConversionFactor(rightLeader1);
    setConversionFactor(rightFollower2);
    setConversionFactor(rightFollower3);
    setMotorControllerSettings(leftLeader1);
    setMotorControllerSettings(leftFollower2);
    setMotorControllerSettings(leftFollower3);
    setMotorControllerSettings(rightLeader1);
    setMotorControllerSettings(rightFollower2);
    setMotorControllerSettings(rightFollower3);
    driveTrain.setDeadband(Constants.Drive.DriveSettings.JOYSTICK_DEADBAND);
    driveTrain.setMaxOutput(Constants.Drive.DriveSettings.MAX_OUTPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCurrentPose();
  }

  public void setConversionFactor(CANSparkMax motor) {
    motor.getEncoder().setPositionConversionFactor(Constants.Drive.ConversionFactors.POSITION_CONVERSION_FACTOR);
    motor.getEncoder().setVelocityConversionFactor(Constants.Drive.ConversionFactors.VELOCITY_CONVERSION_FACTOR);
  }

  private void setMotorControllerSettings(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(Constants.Drive.MotorControllerSettings.DRIVETRAIN_IDLE_MODE);
    motor.setSmartCurrentLimit(Constants.Drive.MotorControllerSettings.DRIVETRAIN_CURRENT_LIMIT);
    motor.burnFlash();
  }

  private void resetEncoder(CANSparkMax motor) {
    motor.getEncoder().setPosition(0);
  }


  public void stop() {
    leftLeader1.stopMotor();
    rightLeader1.stopMotor();
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public void setCurrentPose() {
    driveOdometry.update(RobotContainer.navX.getRotation2d(), leftLeader1.getEncoder().getPosition(),
    rightLeader1.getEncoder().getPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoder(leftLeader1);
    resetEncoder(leftFollower2);
    resetEncoder(leftFollower3);
    resetEncoder(rightLeader1);
    resetEncoder(rightFollower2);
    resetEncoder(rightFollower3);
    driveOdometry.resetPosition(pose, RobotContainer.navX.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeader1.getEncoder().getVelocity(),
        rightLeader1.getEncoder().getVelocity());
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftFFEffort = simpleMotorFeedforward.calculate(leftVelocity);
    var leftPIDEffort = m_leftPIDController.calculate(leftLeader1.getEncoder().getVelocity(), leftVelocity);

    var rightFFEffort = simpleMotorFeedforward.calculate(rightVelocity);
    var rightPIDEffort = m_rightPIDController.calculate(rightLeader1.getEncoder().getVelocity(), rightVelocity);
    SmartDashboard.putNumber("Left Setpoint", leftVelocity);
    SmartDashboard.putNumber("Right Setpoint", rightVelocity);
    SmartDashboard.putNumber("Left State", leftLeader1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right State",rightLeader1.getEncoder().getVelocity());
    leftLeader1.setVoltage(leftFFEffort + leftPIDEffort);
    leftLeader1.setVoltage(rightFFEffort + rightPIDEffort);
  }

  public void arcadeDrive(double forward, double turn) {
    driveTrain.arcadeDrive(forward, turn);
  }

  public void tankDrive(double left, double right) {
    driveTrain.tankDrive(left, right);
  }

  public Command commandForTrajectory(Trajectory trajectory) {

    resetOdometry(trajectory.getInitialPose());
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, this::getPose,
        new RamseteController(Constants.Drive.Auton.kRamseteB, Constants.Drive.Auton.kRamseteZeta),
        Constants.Drive.Auton.driveKinematics, this::tankDriveVelocity, this);

    return ramseteCommand;

  }
}
