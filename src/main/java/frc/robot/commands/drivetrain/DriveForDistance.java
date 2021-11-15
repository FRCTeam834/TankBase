// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForDistance extends CommandBase {
  double distance = 0.0;
  double speed = 0.0;
  double time = 0.0;
  Timer timer = new Timer();
  DriveTrain driveTrain;

  /** Creates a new DriveForDistance. */
  public DriveForDistance(DriveTrain driveTrain, double distance, double speed) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.distance = distance;
    this.speed = speed;
    time = distance / speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.tankDriveVelocity(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
