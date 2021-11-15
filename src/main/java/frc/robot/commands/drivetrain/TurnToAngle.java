// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  PIDController anglePID = new PIDController(.5, 0, 0);
  double targetAngle = 0.0;
  double currentAngle = 0.0;
  private final DriveTrain driveTrain;
  private final AHRS navX;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, AHRS navX, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.navX = navX;
    addRequirements(driveTrain);
    this.targetAngle = targetAngle;
    anglePID.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = navX.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(0.0,
        anglePID.calculate(navX.getYaw(), currentAngle + targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint();
  }
}
