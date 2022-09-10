/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.GyroSubSys;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Components.GyroSubSys.NavX.NavXGyroSubSys;
import frc.robot.Components.GyroSubSys.NavX.NavXGyroSubSys_Constants;
import frc.robot.Components.GyroSubSys.Pigeon.PigeonGyroSubSys;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSubSys extends SubsystemBase {
  /**
   * Creates a new Gyro SubSys
   * 
   *   Select Only One Type
   */

  // NavX
  //private NavXGyroSubSys m_Gyro;

  // Pigeon2
  private PigeonGyroSubSys m_Gyro;

  public GyroSubSys() {
    // NavX 
    //m_Gyro = new NavXGyroSubSys();
    
    // Pigeon2
    m_Gyro = new PigeonGyroSubSys();

    // Reset to 0 on powerup
    m_Gyro.setYaw(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display
    SmartDashboard.putNumber("GyroYawAngleDeg", m_Gyro.getGyroAngle());
  }

  /**
   * Return Gyro Yaw Angle Degrees
   * @return Gyro Yaw Angle double Degrees
   */
  public double getGyroYawAngleDeg(){
    return m_Gyro.getGyroAngle();
  }

  /**
   * Return Gyro Yaw Angle in Rotation2d
   * @return Gyro Yaw Angle Rotation2d
   */
  public Rotation2d getGyroYawRotation2d(){
    return Rotation2d.fromDegrees(getGyroYawAngleDeg());
  }

  /**
   * Set Gyro Yaw Angle
   * @param yawAngleDeg Gyro Yaw Angle Degrees
   */
  public void setYaw(double yawAngleDeg){
    m_Gyro.setYaw(yawAngleDeg);
  }

}
