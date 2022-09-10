/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys;
import frc.robot.Components.GyroSubSys.Pigeon.PigeonGyroSubSys;
import frc.robot.Constants.RobotSettings;

public class DriveSubSys_Old extends SubsystemBase {
  /**
   * Creates a new DriveSubSys.
   */
  
  // Drive Types - Select only 1
  
  // Tank
  //private final TankDriveSubSys m_Drive = 
  //new TankDriveSubSys;

  // Mecanum
  //private final MecanumDriveSubSys m_Drive = 
  //new MecanumDriveSubSys;

  // Swerve
  private SwerveDriveSubSys m_DriveSubSys; 

  // Drive Commands
  private double m_DriveXDirCmd = 0;
  private double m_DriveYDirCmd = 0;
  private double m_DriveZRotCmd = 0;
  private boolean m_DriveFieldOriented = false;
  private boolean m_DriveRotateLeftPt = false;
  private boolean m_DriveRotateRightPt = false;

  // DriveSubSys Shuffleboard
  private DriveSubSys_Shuffleboard m_DriveSubSys_Shuffleboard =
    new DriveSubSys_Shuffleboard();

  // GyroScope
  private PigeonGyroSubSys m_GyroSubSys; 


  /**
   * DriveSubSys Constructor
   * @param gyroSubSys NavXGyroSubSys
   */
  public DriveSubSys_Old(PigeonGyroSubSys gyroSubSys) {
    m_GyroSubSys = gyroSubSys;

    m_DriveSubSys = new SwerveDriveSubSys(m_GyroSubSys);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Send Drive Commands
    m_DriveSubSys.drive(
      m_DriveXDirCmd,
      m_DriveYDirCmd,
      m_DriveZRotCmd,
      m_DriveFieldOriented,
      m_DriveRotateLeftPt,
      m_DriveRotateRightPt);

    SmartDashboard.putNumber("Xdistance", m_DriveSubSys.getPose().getX());
    SmartDashboard.putNumber("Ydistance", m_DriveSubSys.getPose().getY());
    SmartDashboard.putNumber("Heading", m_DriveSubSys.getHeading().getDegrees());

    // Display Drive SubSys Shuffleboard
    if(DriveSubSys_Constants.DriveSubSys_Shuffleboard_Enable){
      m_DriveSubSys_Shuffleboard.Display_DriveSubSys(
        m_GyroSubSys,
        m_DriveXDirCmd,
        m_DriveYDirCmd,
        m_DriveZRotCmd,
        m_DriveFieldOriented,
        m_DriveRotateLeftPt,
        m_DriveRotateRightPt,
        m_DriveSubSys.getPose());
    }
  }

  /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_DriveSubSys.getPose();
    }

    /**
     * Returns the currently heading robot.
     *
     * @return The heading.
     */
    public Rotation2d getHeading() {
      return m_GyroSubSys.getGyroRotation2d();
    }
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      m_DriveSubSys.resetOdometry(pose);
    }

    /**
    * Zeros the Drive Motor encoders
    */
    public void resetDrive() {
      m_DriveSubSys.resetSwerveModules();
    }
    
    /**
    * Resets the Swerve PIDs
    */
    public void resetDrivePIDs(){
      m_DriveSubSys.resetSwerveModulesPID();
    }

    /**
     * Resets the Gyro angle to specified heading
     *
     * @param pose The pose to which to set the odometry.
     */
    public void zeroGyro() {
      m_GyroSubSys.zeroYaw();
    }

     /**
     * Returns Max Chassis Speed
     *
     * @return Max Drive SubSystem Speed (m/s)
     */
    public double getMaxDriveSubSysSpd() {
      return RobotSettings.Drive.MaxDriveSubSysSpd;
    }
  
     /**
     * Returns Max Chassis Acceleration
     *
     * @return Max Drive SubSystem Acceleration (m/s^2)
     */
    public double getMaxDriveSubSysAccel() {
      return RobotSettings.Drive.MaxDriveSubSysAccel;
    }

    /**
     * Returns Max Chassis Rotation speed
     *
     * @return Max Drive SubSystem Rotational speed (rad/s)
     */
    public double getMaxDriveSubSysRotSpd() {
      return RobotSettings.Drive.MaxDriveSubSysRotSpeed;
    }

    /**
     * Returns Max Chassis Rotation Acceleration
     *
     * @return Max Drive SubSystem Rotational Acceleration (rad/s^2)
     */
    public double getMaxDriveSubSysRotAccel() {
      return RobotSettings.Drive.MaxDriveSubSysRotAccel;
    }

     /**
     * Returns Drive SubSystem Kinematics
     *
     * @return Drive SubSystem Kinematics
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
      return m_DriveSubSys.getSwerveDriveKinematics();
    }

    /**
     * Method to drive the robot using setpoint.
     *
     * @param xSpdCmd       Speed Cmd of the robot in the x direction (forward) m/s.
     * @param ySpdCmd       Speed Cmd of the robot in the y direction (sideways) m/s.
     * @param rotSpdCmd     Rotational speed Cmd of the robot. rad/s
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rotateLeftPt  boolean Rotate around Left Pt
     * @param rotateRightPt boolean Rotate around Right Pt
     */
    @SuppressWarnings("ParameterName")
    public void Drive(
      double xSpdCmd,
      double ySpdCmd,
      double rotSpdCmd,
      boolean fieldRelative,
      boolean rotateLeftPt,
      boolean rotateRightPt) {
      
        //Limit Cmds to Chassis Limits
        
        m_DriveXDirCmd = Math.min(Math.max(xSpdCmd,-RobotSettings.Drive.MaxDriveSubSysSpd),RobotSettings.Drive.MaxDriveSubSysSpd);
        m_DriveYDirCmd = Math.min(Math.max(ySpdCmd,-RobotSettings.Drive.MaxDriveSubSysSpd),RobotSettings.Drive.MaxDriveSubSysSpd);
        m_DriveZRotCmd = Math.min(Math.max(rotSpdCmd,-RobotSettings.Drive.MaxDriveSubSysRotSpeed),RobotSettings.Drive.MaxDriveSubSysRotSpeed);
        m_DriveFieldOriented = fieldRelative;
        m_DriveRotateLeftPt = rotateLeftPt;
        m_DriveRotateRightPt = rotateRightPt;
    }
}
