// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class DebugTrajectories extends SequentialCommandGroup {
  private DriveTrain driveTrain;

  /** Creates a new DebugTrajectories. */
  public DebugTrajectories(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    addCommands(new RunCommand(() -> driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0))), driveTrain),
        new PrintCommand("The current robot pose is: \nX:" + driveTrain.getPose().getX() + "\nY: " + driveTrain.getPose().getY() + "\nAngle: " + driveTrain.getPose().getRotation()),
        new PrintCommand("The robot will now attempt to drive one meter forward."),
        new ExampleTrajectory(driveTrain),
        new PrintCommand("The robot should have traveled one meter forward while keeping it's current angle."),
        new PrintCommand("The robot thinks that it is at: \nX:" + driveTrain.getPose().getX() + "\nY: " + driveTrain.getPose().getY() + "\nAngle: " + driveTrain.getPose().getRotation()));
  }
}
