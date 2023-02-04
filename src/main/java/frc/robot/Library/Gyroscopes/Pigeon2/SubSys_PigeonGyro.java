/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.Gyroscopes.Pigeon2;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Library.Gyroscopes.NavX.SubSys_NavXGyro_Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class SubSys_PigeonGyro extends SubsystemBase {
  /**
   * Creates a new NavXGyro.
   */
  private Pigeon2 m_Pigeon2Gyro;

  public SubSys_PigeonGyro() {
    m_Pigeon2Gyro = new Pigeon2(Constants.CAN_IDs.Pigeon2_ID);
    m_Pigeon2Gyro.setYaw(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display
    SmartDashboard.putNumber("GyroYaw", m_Pigeon2Gyro.getYaw());
    SmartDashboard.putNumber("GyroCompass", m_Pigeon2Gyro.getCompassHeading());
  }
  public double getRawGyroPitch(){
    return m_Pigeon2Gyro.getPitch();
  }

  public double getRawGyroRoll(){
    return m_Pigeon2Gyro.getRoll();
  }
  /**
   * Return Raw Gyro Angle
   * @return Raw Angle double raw Gyro Angle
   */
  public double getRawGyroAngle(){
    return m_Pigeon2Gyro.getYaw();
  }

  /**
   * Return Gyro Angle in Degrees
   * @return Gyro Angle double Degrees
   */
  public double getGyroAngle(){
    return Math.IEEEremainder(m_Pigeon2Gyro.getYaw(), 360) * (SubSys_NavXGyro_Constants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Return Gyro Angle in Rotation2d
   * @return Gyro Angle Rotation2d
   */
  public Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(getGyroAngle());
  }

   /**
   * Zero Gyro
   */
  public void zeroYaw(){
    m_Pigeon2Gyro.setYaw(0);
  }

  /**
   * Set Gyro
   */
  public void setYaw(double yawDegrees){
    m_Pigeon2Gyro.setYaw(yawDegrees);
  }

}
