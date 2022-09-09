/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Components.GyroSubSys.NavX.NavXGyroSubSys;
import frc.robot.Components.GyroSubSys.Pigeon.PigeonGyroSubSys;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/**
 * Add your docs here.
 */
public class DriveSubSys_Shuffleboard {
    
  // Create Swerve Module Tab
  //String tabTitle = new String(swerveModuleStringID);
  //tabTitle.concat("_SwerveModule");
  //ShuffleboardTab tab = Shuffleboard.getTab(tabTitle); 
  private ShuffleboardTab m_ShflTab_DriveSubSys;
  private NetworkTableEntry m_DriveXDirCmd;
  private NetworkTableEntry m_DriveYDirCmd;
  private NetworkTableEntry m_DriveZRotCmd;
  private NetworkTableEntry m_Gyro;
  private NetworkTableEntry m_DriveRotateLeftPt;
  private NetworkTableEntry m_DriveRotateRightPt;
  private NetworkTableEntry m_PoseX;
  private NetworkTableEntry m_PoseY;
  private NetworkTableEntry m_PoseRot;

  /**
   * Constructor for DriveSubSys Shuffleboard Tab
   * 
   */
  public DriveSubSys_Shuffleboard(){

    m_ShflTab_DriveSubSys = Shuffleboard.getTab("DriveSubSys");  

    m_DriveXDirCmd = m_ShflTab_DriveSubSys.add("DriveXDirCmd", 99)
      .withSize(6, 2)
      .withPosition(1, 3)
      .getEntry();

    m_DriveYDirCmd = m_ShflTab_DriveSubSys.add("DriveYDirCmd", 99)
      .withSize(6, 2)
      .withPosition(1, 6)
      .getEntry();

    m_DriveZRotCmd = m_ShflTab_DriveSubSys.add("DriveZRotCmd", 99)
      .withSize(6, 2)
      .withPosition(1, 9)
      .getEntry();

    m_DriveRotateLeftPt = m_ShflTab_DriveSubSys.add("Drive Rot Left Pt", 0)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(6, 2)
      .withPosition(1, 12)
      .getEntry();

    m_DriveRotateRightPt = m_ShflTab_DriveSubSys.add("Drive Rot Right Pt", 0)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(6, 2)
      .withPosition(1, 15)
      .getEntry();

    m_Gyro = m_ShflTab_DriveSubSys.add("Gyro", 0)
      .withSize(6, 2)
      .withPosition(8, 3)
      .getEntry();  

    m_PoseX = m_ShflTab_DriveSubSys.add("Pose X", 0)
      .withSize(6, 2)
      .withPosition(8, 6)
      .getEntry();  

    m_PoseY = m_ShflTab_DriveSubSys.add("Pose Y", 0)
      .withSize(6, 2)
      .withPosition(8, 9)
      .getEntry(); 

    m_PoseRot = m_ShflTab_DriveSubSys.add("Pose Rot", 0)
      .withSize(6, 2)
      .withPosition(8, 12)
      .getEntry(); 
  }


  /**
   * Drive SubSystem Shuffleboard Display
   * @param gyroSubSys    GyroscopeSubSystem
   * @param driveXDirCmd  double X direction Command
   * @param driveYDirCmd  double Y direction Command
   * @param driveZRotCmd  double Z Rotation Command
   * @param driveFieldOriented boolean Field Oriented Enabled
   * @param driveRotateLeftPt boolean Rotate around Left Pt
   * @param driveRotateRightPt boolean Rotate around Right Pt
   * @param drivePose2d Pose2d Drive Subsystem Pose
   */
  public void Display_DriveSubSys(
    PigeonGyroSubSys gyroSubSys,
    double driveXDirCmd,
    double driveYDirCmd,
    double driveZRotCmd,
    boolean driveFieldOriented,
    boolean driveRotateLeftPt,
    boolean driveRotateRightPt,
    Pose2d drivePose2d){

    // Update Drive Commands
    m_DriveXDirCmd.setDouble(driveXDirCmd);
    m_DriveYDirCmd.setDouble(driveYDirCmd);
    m_DriveZRotCmd.setDouble(driveZRotCmd);
    m_Gyro.setDouble(gyroSubSys.getGyroRotation2d().getDegrees());
    m_DriveRotateLeftPt.setBoolean(driveRotateLeftPt);
    m_DriveRotateRightPt.setBoolean(driveRotateRightPt);
    m_PoseX.setDouble(drivePose2d.getX());
    m_PoseY.setDouble(drivePose2d.getY());
    m_PoseRot.setDouble(drivePose2d.getRotation().getDegrees());
  }
}
