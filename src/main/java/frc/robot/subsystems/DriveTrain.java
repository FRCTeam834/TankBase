// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.RobotContainer;


public class DriveTrain extends SubsystemBase {
  //Left side of the drivetrain

  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_1, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_2, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(Constants.Drive.DriveMotors.LEFT_MOTOR_3, CANSparkMax.MotorType.kBrushless);

  SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMotor1, m_leftMotor2,m_leftMotor3);

  //Right side of the drivetrain
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_1, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_2, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(Constants.Drive.DriveMotors.RIGHT_MOTOR_3, CANSparkMax.MotorType.kBrushless);

  SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  //Differential Drive
  private final DifferentialDrive driveTrain = new DifferentialDrive(m_leftMotors,m_rightMotors);
  private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(RobotContainer.navX.getRotation2d());

  private final PIDController m_leftPIDController = new PIDController(Constants.Drive.Auton.kPDriveVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(Constants.Drive.Auton.kPDriveVel, 0, 0);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    setMotorInversions();
    setConversionFactor(m_leftMotor1);
    setConversionFactor(m_leftMotor2);
    setConversionFactor(m_leftMotor3);
    setConversionFactor(m_rightMotor1);
    setConversionFactor(m_rightMotor2);
    setConversionFactor(m_rightMotor3);
    setMotorControllerSettings(m_leftMotor1);
    setMotorControllerSettings(m_leftMotor2);
    setMotorControllerSettings(m_leftMotor3);
    setMotorControllerSettings(m_rightMotor1);
    setMotorControllerSettings(m_rightMotor2);
    setMotorControllerSettings(m_rightMotor3);
    configureDrive(Constants.Drive.DriveSettings.JOYSTICK_DEADBAND, Constants.Drive.DriveSettings.MAX_OUTPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCurrentPose();
  }

  public void setConversionFactor(CANSparkMax motor)
  {
    motor.getEncoder().setPositionConversionFactor(Constants.Drive.ConversionFactors.POSITION_CONVERSION_FACTOR);
    motor.getEncoder().setVelocityConversionFactor(Constants.Drive.ConversionFactors.VELOCITY_CONVERSION_FACTOR);
  }
  private void setMotorInversions()
  {
    m_leftMotors.setInverted(Constants.Drive.DriveMotors.LEFT_INVERTED);
    m_leftMotors.setInverted(Constants.Drive.DriveMotors.RIGHT_INVERTED);
  }

  private void setMotorControllerSettings(CANSparkMax motor)
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(Constants.Drive.MotorControllerSettings.DRIVETRAIN_IDLE_MODE);
    motor.setSmartCurrentLimit(Constants.Drive.MotorControllerSettings.DRIVETRAIN_CURRENT_LIMIT);
    motor.burnFlash();
  }

  private void resetEncoder(CANSparkMax motor)
  {
    motor.getEncoder().setPosition(0);
  }

  private void configureDrive(double deadband, double maxOutput)
  {
    driveTrain.setDeadband(deadband);
    driveTrain.setMaxOutput(maxOutput);
  }

  public void stop()
  {
    m_leftMotors.stopMotor();
    m_rightMotors.stopMotor();
  }

  public Pose2d getPose()
  {
    return driveOdometry.getPoseMeters();
  }

  public void setCurrentPose()
  {
    driveOdometry.update(
      RobotContainer.navX.getRotation2d(), 
      m_leftMotor1.getEncoder().getPosition(), 
      m_rightMotor1.getEncoder().getPosition()
      );
  }

  public void resetOdometry(Pose2d pose)
  {
    resetEncoder(m_leftMotor1);
    resetEncoder(m_leftMotor2);
    resetEncoder(m_leftMotor3);
    resetEncoder(m_rightMotor1);
    resetEncoder(m_rightMotor2);
    resetEncoder(m_rightMotor3);
    driveOdometry.resetPosition(pose, RobotContainer.navX.getRotation2d());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(
      m_leftMotor1.getEncoder().getVelocity(), 
      m_rightMotor1.getEncoder().getVelocity()
      );
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity)
  {
    m_leftMotors.set(MathUtil.clamp(m_leftPIDController.calculate(m_leftMotor1.getEncoder().getVelocity(), leftVelocity), -Constants.Drive.Auton.MAX_SPEED, Constants.Drive.Auton.MAX_SPEED));
    m_rightMotors.set(MathUtil.clamp(m_rightPIDController.calculate(m_rightMotor1.getEncoder().getVelocity(), rightVelocity), -Constants.Drive.Auton.MAX_SPEED, Constants.Drive.Auton.MAX_SPEED));

  }

  public void arcadeDrive(double forward, double turn)
  {
    driveTrain.arcadeDrive(forward, turn);
  }

  public void tankDrive(double left, double right)
  {
    driveTrain.tankDrive(left, right);
  }

  public Command commandForTrajectory(Trajectory trajectory)
  {
    
    resetOdometry(trajectory.getInitialPose());
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      this::getPose,
      new RamseteController(Constants.Drive.Auton.kRamseteB,Constants.Drive.Auton.kRamseteZeta),
      Constants.Drive.Auton.driveKinematics,
      this::tankDriveVelocity,
      this);

      return ramseteCommand;
      
  }
  
  
}
