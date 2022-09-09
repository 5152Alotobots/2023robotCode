/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1.SwerveModule_Type_1_SubSys;

public class SwerveModuleSubSys extends SubsystemBase {
  /**
   * Creates a new SwerveModule.
   * 
   */
  public SwerveModule_Type_1_SubSys
    m_SwerveModule; 

  /**
   * Creates a new SwerveModule.
   * 
   * @param swerveModuleID            String  Swerve Module ID (FL,FR,RL,RR) 
   * @param drvMtr_ID                 int     ID for the drive motor
   * @param strMtr_ID                 int     ID for the turning motor
   */  
  public SwerveModuleSubSys(
    String swerveModuleID,          
    int drvMtr_ID,                 
    int strMtr_ID)                  
  {
    m_SwerveModule = new SwerveModule_Type_1_SubSys(
      swerveModuleID,
      drvMtr_ID,
      strMtr_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state SwerveModuleState:  Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    m_SwerveModule.setDesiredState(state);
  }

  /**
   * Returns the current state of the module.
   *
   * @return SwerveModuleState:  The current state of the module.
   */
  public SwerveModuleState getState() {
    return m_SwerveModule.getState();
  }

  /**
   * Get Max Drive Wheel Speed
   * @return maxDriveMtrWheelSpd double Maximum Drive Wheel Speed (m/s)
   */
  public double getMaxDriveWheelSpd(){
    return m_SwerveModule.getMaxDriveWheelSpd();
  }

  /**
  * Zeros the Swerve Drive Motor encoder.
  */
  public void resetSwerveModule() {
    m_SwerveModule.resetSwerveModule();
  }

  /**
   * Resets the Swerve PIDs
   */
  public void resetSwerveModulePID(){
    m_SwerveModule.resetSwerveModulePID();
  }
}
