// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ExampleTrajectory extends SequentialCommandGroup {
  private DriveTrain driveTrain;

  /** Creates a new ExampleTrajectory. */
  public ExampleTrajectory(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    Rotation2d currentAngle = RobotContainer.navX.getRotation2d();
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, currentAngle),
        List.of(new Translation2d(1, 0)),
        new Pose2d(1, 0, currentAngle),
        Constants.Drive.TrajectorySettings.config);
    addCommands(driveTrain.commandForTrajectory(exampleTrajectory).andThen(driveTrain::stop));
  }
}
