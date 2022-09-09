/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/**
 * Add your docs here.
 */
public class SwerveModule_Type_1_Shuffleboard {
    
  private String m_SwerveModuleStringID;             
  private ShuffleboardTab m_ShflTab_SwModule; 

  // Drive Motor
  private NetworkTableEntry m_DrvMtr_RawSensorPos;
  private NetworkTableEntry m_DrvMtr_Spd;
  private NetworkTableEntry m_DrvMtr_SpdCmd;
  private NetworkTableEntry m_DrvMtr_RawSpdCmd;
  private NetworkTableEntry m_DrvMtr_PID_TestEnable;
  private NetworkTableEntry m_DrvMtr_P;
  private NetworkTableEntry m_DrvMtr_I;
  private NetworkTableEntry m_DrvMtr_D;
  
  // Steer Motor
  private NetworkTableEntry m_StrMtr_RawSensorPos;
  private NetworkTableEntry m_StrMtr_Pos;
  private NetworkTableEntry m_StrMtr_PosCmd;
  private NetworkTableEntry m_StrMtr_RawPosCmd;
  private NetworkTableEntry m_StrMtr_PID_TestEnable;
  private NetworkTableEntry m_StrMtr_P;
  private NetworkTableEntry m_StrMtr_I;
  private NetworkTableEntry m_StrMtr_D;
  

  /**
   * Constructor for Swerve Module Shuffleboard Tab
   * @param swerveModuleStringID String Swerve Module String ID
   */
  public SwerveModule_Type_1_Shuffleboard(
    String swerveModuleStringID){
      m_SwerveModuleStringID = swerveModuleStringID;
    
      // Create New Shuffleboard Tab
      String TabName = m_SwerveModuleStringID.concat("_SW_Module");
      m_ShflTab_SwModule = Shuffleboard.getTab(TabName);  
      
      // Create Drive Motor Layout
      //m_ShflLayout_Drive = m_ShflTab_SwModule.getLayout("Drive Motor", BuiltInLayouts.kList)
      //  .withSize(6, 15)
      //  .withPosition(0, 0)
      //  .withProperties(Map.of("Label position","TOP","Number of columns", 1));

      // Drive Motor Basic Data  
      m_DrvMtr_RawSensorPos = m_ShflTab_SwModule.add("Drive RawSensor", 0)
        .withSize(10, 5)
        .withPosition(0, 0)
        .getEntry();  

      m_DrvMtr_Spd = m_ShflTab_SwModule.add("Drive Speed", 0)
        .withSize(10, 5)
        .withPosition(0, 5)
        .getEntry(); 

      m_DrvMtr_SpdCmd = m_ShflTab_SwModule.add("Drive Speed Cmd", 0)
        .withSize(10, 5)
        .withPosition(0, 10)
        .getEntry(); 

      m_DrvMtr_RawSpdCmd = m_ShflTab_SwModule.add("Drive Raw Speed Cmd", 0)
        .withSize(10, 5)
        .withPosition(0, 10)
        .getEntry();  

      // Drive Motor PID Tuning
      m_DrvMtr_PID_TestEnable = m_ShflTab_SwModule.add("Drive PID Test Enable", false)
        .withSize(10, 5)
        .withPosition(1, 15)        
        .getEntry();

      m_DrvMtr_P = m_ShflTab_SwModule.add("Drive P Gain", 0)
        .withSize(10, 5)
        .withPosition(1, 20)        
        .getEntry();

      m_DrvMtr_I = m_ShflTab_SwModule.add("Drive I Gain", 0)
        .withSize(10, 5)
        .withPosition(1, 25)        
        .getEntry();  

      m_DrvMtr_D = m_ShflTab_SwModule.add("Drive D Gain", 0)
        .withSize(10, 5)
        .withPosition(1, 30)        
        .getEntry();  

      // Create Steer Motor Layout
      //m_ShflLayout_Steer = m_ShflTab_SwModule.getLayout("Steer Motor", BuiltInLayouts.kGrid)
      //  .withSize(30,30)
      //  .withPosition(20, 0)
      //  .withProperties(Map.of("Label position","TOP", "Number of columns", 1));

      m_StrMtr_RawSensorPos = m_ShflTab_SwModule.add("Steer Raw Sensor", 0)
        .withSize(6,2)
        .withPosition(20, 0)
        .getEntry();

      m_StrMtr_Pos = m_ShflTab_SwModule.add("Steer Pos", 0)
        .withSize(6,2)
        .withPosition(20, 2)
        .getEntry(); 

      m_StrMtr_PosCmd = m_ShflTab_SwModule.add("Steer Pos Cmd", 0)
        .withSize(6,2)
        .withPosition(26, 4)
        .getEntry();

      m_StrMtr_RawPosCmd = m_ShflTab_SwModule.add("Steer Raw Pos Cmd", 0)
        .withSize(6,2)
        .withPosition(20, 4)
        .getEntry();

        
      // Steer Motor PID Tuning
      m_StrMtr_PID_TestEnable = m_ShflTab_SwModule.add("Steer PID Test Enable", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(6,2)
        .withPosition(20, 6)        
        .getEntry();

      m_StrMtr_P = m_ShflTab_SwModule.add("Steer P Gain", 0)
        .withSize(6,2)
        .withPosition(20, 8)        
        .getEntry();

      m_StrMtr_I = m_ShflTab_SwModule.add("Steer I Gain", 0)
        .withSize(6,2)
        .withPosition(20, 10)        
        .getEntry();  

      m_StrMtr_D = m_ShflTab_SwModule.add("Steer D Gain", 0)
        .withSize(6,2)
        .withPosition(20, 12)        
        .getEntry();   
  }

  /**
   * Displays Swerve Module Information
   * @param drvMtr TalonFX Drive Motor
   * @param strMtr TalonSRX Steet Motor
   * @param sensorZeroPosCnts int Counts of Zero Position on Encoder
   * @param strMtrPID PIDController Steer Motor PID object
   * @param strMtrSetpoint double Setpoint of Steer Motor in Radians
   * @param strPIDMtrCmd double Steer Motor PID cmd in percent output
   * @param swerveModuleStringID String Swerve Module String ID ex. (FL, FR, RL, RR)
   * @param swerveModuleState SwerveModuleState State of Swerve Elements
   * @param swerveModuleStateCmd SwerveModuleState Commanded State for Swerve Elements
   * @param optimumSwerveModuleStateCmd Optimized Serve Module Commanded State
  */
  public void Display_SwerveModule(
    String swerveModuleStringID,
    TalonFX drvMtr,
    TalonSRX strMtr,
    int sensorZeroPosCnts,
    PIDController strMtrPID,
    double strMtrSetpoint,
    double strPIDMtrCmd,
    SwerveModuleState swerveModuleState,
    SwerveModuleState swerveModuleStateCmd,
    SwerveModuleState optimumSwerveModuleStateCmd){
  
      // Update Drive Motor
      m_DrvMtr_RawSensorPos.setDouble(drvMtr.getSelectedSensorPosition());
      m_DrvMtr_Spd.setDouble(swerveModuleState.speedMetersPerSecond);
      m_DrvMtr_RawSpdCmd.setDouble(swerveModuleStateCmd.speedMetersPerSecond);
      m_DrvMtr_SpdCmd.setDouble(optimumSwerveModuleStateCmd.speedMetersPerSecond);
      
      // Update Steer Motor
      m_StrMtr_RawSensorPos.setDouble(strMtr.getSelectedSensorPosition());
      m_StrMtr_Pos.setDouble(swerveModuleState.angle.getRadians());
      m_StrMtr_RawPosCmd.setDouble(swerveModuleStateCmd.angle.getRadians());
      m_StrMtr_PosCmd.setDouble(optimumSwerveModuleStateCmd.angle.getRadians());
  } 

  /**
  * Get Drive Motor PID Tuning Enable
  * @return boolean PID Enable
  */
  public boolean getDrvMtrPID_TestEnable(){
    return m_DrvMtr_PID_TestEnable.getBoolean(false);
  }

  /**
  * Get Drive Motor PID P gain
  * @return double P gain
  */
  public double getDrvMtrPID_P(){
    return m_DrvMtr_P.getDouble(0);
  }

  /**
  * Get Drive Motor PID I gain
  * @return double I gain
  */
  public double getDrvMtrPID_I(){
    return m_DrvMtr_I.getDouble(0);
  }
    
  /**
  * Get Drive Motor PID D gain
  * @return double D gain
  */
  public double getDrvMtrPID_D(){
    return m_DrvMtr_D.getDouble(0);
  }

  /**
  * Get Steer Motor PID Tuning Enable
  * @return boolean PID Enable
  */
  public boolean getStrMtrPID_TestEnable(){
    return m_StrMtr_PID_TestEnable.getBoolean(false);
  }

  /**
  * Get Steer Motor PID P gain
  * @return double P gain
  */
  public double getStrMtrPID_P(){
    return m_StrMtr_P.getDouble(0);
  }

  /**
  * Get Steer Motor PID I gain
  * @return double I gain
  */
  public double getStrMtrPID_I(){
    return m_StrMtr_I.getDouble(0);
  }
    
  /**
  * Get Steer Motor PID D gain
  * @return double D gain
  */
  public double getStrMtrPID_D(){
    return m_StrMtr_D.getDouble(0);
  }
}
