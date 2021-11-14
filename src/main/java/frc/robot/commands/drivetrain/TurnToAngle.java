// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnToAngle extends CommandBase {
  PIDController anglePID = new PIDController(.5, 0, 0);
  double targetAngle = 0.0;
  double currentAngle = 0.0;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.navX,RobotContainer.driveTrain);
    this.targetAngle = targetAngle;
    anglePID.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = RobotContainer.navX.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveTrain.arcadeDrive(0, anglePID.calculate(RobotContainer.navX.getYaw(), currentAngle + targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint();
  }
}
