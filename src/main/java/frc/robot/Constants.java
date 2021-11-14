// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * Constants used for the drivetrain
 * @param LEFT_MOTOR_1 CAN ID for left motor 1
 * @param LEFT_MOTOR_2 CAN ID for left motor 2
 * @param LEFT_MOTOR_3 CAN ID for left motor 3
 * @param RIGHT_MOTOR_1 CAN ID for right motor 1
 * @param RIGHT_MOTOR_2 CAN ID for right motor 2
 * @param RIGHT_MOTOR_3 CAN ID for right motor 3
 */
    public static final class Drive {
        public static final class DriveMotors
        {
            public static final int LEFT_MOTOR_1 = 0;
            public static final int LEFT_MOTOR_2 = 0;
            public static final int LEFT_MOTOR_3 = 0;
    
            public static final int RIGHT_MOTOR_1 = 0;
            public static final int RIGHT_MOTOR_2 = 0;
            public static final int RIGHT_MOTOR_3 = 0;
    
            public static final boolean LEFT_INVERTED = false;
            public static final boolean RIGHT_INVERTED = true;

        }

        public static final class ConversionFactors
        {
            public static final double GEARING = 0.0;
            public static final double WHEEL_RADIUS = Units.inchesToMeters(5.0 / 2.0);
    
            public static final double POSITION_CONVERSION_FACTOR = (1 / GEARING) * (2 * Math.PI * WHEEL_RADIUS);
            public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
        }

        public static final class MotorControllerSettings
        {
            public static final IdleMode DRIVETRAIN_IDLE_MODE = IdleMode.kBrake;

            public static final int DRIVETRAIN_CURRENT_LIMIT = 40;
        }

        public static final class DriveSettings
        {
            public static final double JOYSTICK_DEADBAND = .1;
            public static final double MAX_OUTPUT = 1;
        }

        public static final class Auton
        {
           public static final double kRamseteB = 0.0;
           public static final double kRamseteZeta = 0.0;

           public static final double ksVolts = 0.0;
           public static final double kvVoltSecondsPerMeter = 0.0;
           public static final double kaVoltSecondsSquaredPerMeter = 0.0;

           public static final double trackWidth = 0.0;
           public final static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);

           public static final double kPDriveVel =0.0;

           public static final double MAX_SPEED = 0.0;
           
           public static final double MAX_ACCELERATION = 0.0;


        }
        public static final class TrajectorySettings
        {
            DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Drive.Auton.ksVolts,
                Drive.Auton.kvVoltSecondsPerMeter,
                Drive.Auton.kaVoltSecondsSquaredPerMeter),
            Drive.Auton.driveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(Drive.Auton.MAX_SPEED,
                             Drive.Auton.MAX_ACCELERATION)
            .setKinematics(Drive.Auton.driveKinematics)
            .addConstraint(autoVoltageConstraint);
        }
    }

}
