package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */
  //private final DifferentialDriveOdometry m_Odometry;

  AHRS navX = new AHRS(SPI.Port.kMXP);

  public NavX() {
  }

  public float getYaw() {

    return navX.getYaw();

  }

  public float getCompassHeading() {

    return navX.getCompassHeading();

  }

  public Rotation2d getRotation2d()
  {
    return navX.getRotation2d();
  }
  public void resetYaw() {

    navX.reset();

  }

}
