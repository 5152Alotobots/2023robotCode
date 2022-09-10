/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.SwerveModuleSubSys;
import frc.robot.Components.GyroSubSys.Pigeon.PigeonGyroSubSys;
import frc.robot.Constants.CAN_IDs;

public class SwerveDriveSubSys extends SubsystemBase {

  /**
   * Creates a new SwerveDriveSubSys.
   */

  // GyroScope
  private PigeonGyroSubSys m_GyroSubSys;

  // Swerve Modules
  private final SwerveModuleSubSys m_FrontLeftSwModule;
  private final SwerveModuleSubSys m_FrontRightSwModule;
  private final SwerveModuleSubSys m_RearLeftSwModule;
  private final SwerveModuleSubSys m_RearRightSwModule;       
  
  // Odometry class for tracking robot pose
  public SwerveDriveOdometry m_Odometry =
    new SwerveDriveOdometry(SwerveDriveSubSys_Constants.DriveKinematics, Rotation2d.fromDegrees(0.0));

  // Rotate About Point
  private Translation2d m_RotationPt;
  private boolean m_RotateLeftPt_prev;
  private boolean m_RotateRightPt_prev;

  public SwerveDriveSubSys(PigeonGyroSubSys gyroSubSys) {
    m_GyroSubSys = gyroSubSys;

    // Initializize Swerve Modules
    m_FrontLeftSwModule = new SwerveModuleSubSys(
      "FL",                                       // String Module ID
      CAN_IDs.FrontLeftDriveMtr_ID,               // int Drive Mtr Port
      CAN_IDs.FrontLeftSteerMtr_ID);              // int Turning Mtr Port
  
    m_FrontRightSwModule = new SwerveModuleSubSys(
      "FR",                                       // String Module ID
      CAN_IDs.FrontRightDriveMtr_ID,              // int Drive Mtr Port
      CAN_IDs.FrontRightSteerMtr_ID);             // int Turning Mtr Port

    m_RearLeftSwModule = new SwerveModuleSubSys(
      "RL",                                       // String Module ID
      CAN_IDs.RearLeftDriveMtr_ID,                // int Drive Mtr Port
      CAN_IDs.RearLeftSteerMtr_ID);               // int Turning Mtr Port
    
    m_RearRightSwModule = new SwerveModuleSubSys(
      "RR",                                       // String Module ID
      CAN_IDs.RearRightDriveMtr_ID,               // int Drive Mtr Port
      CAN_IDs.RearRightSteerMtr_ID);              // int Turning Mtr Port

    m_RotationPt = new Translation2d(0,0);
    m_RotateLeftPt_prev = false;
    m_RotateRightPt_prev = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     m_Odometry.update(m_GyroSubSys.getGyroRotation2d(),m_FrontLeftSwModule.getState(),m_FrontRightSwModule.getState(),m_RearLeftSwModule.getState(),m_RearRightSwModule.getState());
     SmartDashboard.putNumber("FLSteerRawEnc", m_FrontLeftSwModule.m_SwerveModule.getSteerRawPos());
     SmartDashboard.putNumber("FRSteerRawEnc", m_FrontRightSwModule.m_SwerveModule.getSteerRawPos());
     SmartDashboard.putNumber("RRSteerRawEnc", m_RearRightSwModule.m_SwerveModule.getSteerRawPos());
     SmartDashboard.putNumber("RLSteerRawEnc", m_RearLeftSwModule.m_SwerveModule.getSteerRawPos());
  }

  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }
  
  /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    m_GyroSubSys.setYaw(pose.getRotation().getDegrees());
    m_Odometry.resetPosition(pose, getHeading());
  }

  /**
  * Returns the heading robot.
  *
  * @return The heading.
  */
  public Rotation2d getHeading() {
    return m_GyroSubSys.getGyroRotation2d();
  }

  /**
  * Returns Drive SubSystem Kinematics
  *
  * @return Drive SubSystem Kinematics
  */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return SwerveDriveSubSys_Constants.DriveKinematics;
  }

  /**
  * Zeros the Swerve Drive Motor encoder.
  */
  public void resetSwerveModules() {
    m_FrontLeftSwModule.resetSwerveModule();
    m_FrontRightSwModule.resetSwerveModule();
    m_RearLeftSwModule.resetSwerveModule();
    m_RearRightSwModule.resetSwerveModule();
  }

  /**
   * Resets the Swerve PIDs
   */
  public void resetSwerveModulesPID(){
    m_FrontLeftSwModule.resetSwerveModulePID();
    m_FrontRightSwModule.resetSwerveModulePID();
    m_RearLeftSwModule.resetSwerveModulePID();
    m_RearRightSwModule.resetSwerveModulePID();
  }

  /**
   * Method to drive the robot
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rotateLeftPt  Rotate around Left Pt
   * @param rotateRightPt Rotate around Right Pt
   */
  @SuppressWarnings("ParameterName")
  public void drive(
    double xSpeed,
    double ySpeed,
    double rot,
    boolean fieldRelative,
    boolean rotateLeftPt,
    boolean rotateRightPt) {


    // If Rotate Point Buttons are different than previous commands, create new rotation point.
    if((rotateLeftPt != m_RotateLeftPt_prev) || (rotateRightPt != m_RotateRightPt_prev)){
      
      // Check for only Rotate Left Cmd
      if(rotateLeftPt && !rotateRightPt){
        if (fieldRelative){
          // Facing Forward (Downfield) ~ 350-360 degrees (Use Non-Field Relative logic)
          if (m_GyroSubSys.getGyroRotation2d().getRadians() >= (2*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
            }
          // Facing Forward Right ~ 280-350 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1.5*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
          // Facing Right ~ 260-280 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1.5*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }
          // Facing Rear Right ~ 190-260 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;  
          // Facing Rear ~ 170-190 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
            }
          // Facing Rear Left ~ 100-170 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (0.5*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;  
          // Facing Left ~ 80-100 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (0.5*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
            }
          // Facing Forward Left ~ 10-80 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;  
          // Facing Forward ~ 0-10 degrees 
          }else{
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
            }            
          }
        // Not Field Relative
        }else{
          if(xSpeed>=0){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
          }else{
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
          }
        }

      // Check for only Rotate Right Cmd
      }else if(!rotateLeftPt && rotateRightPt){
        if (fieldRelative){
          // Facing Forward (Downfield) ~ 350-360 degrees (Use Non-Field Relative logic)
          if (m_GyroSubSys.getGyroRotation2d().getRadians() >= (2*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }
          // Facing Forward Right ~ 280-350 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1.5*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
          // Facing Right ~ 260-280 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1.5*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }
          // Facing Rear Right ~ 190-260 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;  
          // Facing Rear ~ 170-190 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (1*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFL;
            }
          // Facing Rear Left ~ 100-170 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (0.5*Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;  
          // Facing Left ~ 80-100 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (0.5*Math.PI-SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRL;
            }
          // Facing Forward Left ~ 10-80 degrees
          }else if(m_GyroSubSys.getGyroRotation2d().getRadians() >= (Math.PI+SwerveDriveSubSys_Constants.RotateFieldRativeMargin)){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;  
          // Facing Forward ~ 0-10 degrees 
          }else{
            if(xSpeed>=0){
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
            }else{
              m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
            }            
          }
        }else{
          if(xSpeed>=0){
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtFR;
          }else{
            m_RotationPt = SwerveDriveSubSys_Constants.RotationPtRR;
          }
        }
      // If not Rotate Pt Cmds or BOTH Rotate Pt Cmds set Rotation Point to Center
      }else{
        m_RotationPt = new Translation2d(0,0);
      }

      // Set Status of RotatePt Buttons for next loop
      m_RotateLeftPt_prev = rotateLeftPt;
      m_RotateRightPt_prev = rotateRightPt;
    }


    var swerveModuleStates = SwerveDriveSubSys_Constants.DriveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-90))
        xSpeed, ySpeed, rot, m_GyroSubSys.getGyroRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot),
      m_RotationPt);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      m_FrontLeftSwModule.getMaxDriveWheelSpd());
  
    m_FrontLeftSwModule.setDesiredState(swerveModuleStates[0]);
    m_FrontRightSwModule.setDesiredState(swerveModuleStates[1]);
    m_RearLeftSwModule.setDesiredState(swerveModuleStates[2]);
    m_RearRightSwModule.setDesiredState(swerveModuleStates[3]);
  }
  
  public void setSpdDir(double spdCmd, double dirCmd){
    // Set Module state
    SwerveModuleState tempModuleState = new SwerveModuleState();
    tempModuleState.speedMetersPerSecond = spdCmd;
    Rotation2d tempAngleCmd = new Rotation2d(dirCmd);
    tempModuleState.angle = tempAngleCmd;

    m_FrontLeftSwModule.setDesiredState(tempModuleState);
    m_FrontRightSwModule.setDesiredState(tempModuleState);
    m_RearLeftSwModule.setDesiredState(tempModuleState);
    m_RearRightSwModule.setDesiredState(tempModuleState);
  }

}
