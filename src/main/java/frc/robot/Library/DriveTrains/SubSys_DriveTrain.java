/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.DriveTrains;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSettings;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_SwerveDrive;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

public class SubSys_DriveTrain extends SubsystemBase {
  /**
   * Creates a new Drive SubSystem.
   */
  
  // Drive Types - Select only 1
  
  // Tank
  //private final TankDriveSubSys m_Drive = 
  //new TankDriveSubSys;

  // Mecanum
  //private final MecanumDriveSubSys m_Drive = 
  //new MecanumDriveSubSys;

  // Swerve
  private SubSys_SwerveDrive driveTrain; 

  // Drive Commands
  private double driveXDirCmd = 0;
  private double driveYDirCmd = 0;
  private double driveZRotCmd = 0;
  private boolean driveFieldOriented = false;
  private boolean driveRotateLeftPtCmd = false;
  private boolean driveRotateRightPtCmd = false;

  // DriveSubSys Shuffleboard
  //private DriveSubSys_Shuffleboard m_DriveSubSys_Shuffleboard =
  //  new DriveSubSys_Shuffleboard();

  // GyroScope
  private SubSys_PigeonGyro gyroSubSys; 


  /**
   * DriveSubSys Constructor
   * @param gyroSubSys SubSys_PigeonGyro
   */
  public SubSys_DriveTrain(SubSys_PigeonGyro gyroSubSys) {
    this.gyroSubSys = gyroSubSys;
    this.driveTrain = new SubSys_SwerveDrive(this.gyroSubSys);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Send Drive Commands
    driveTrain.drive(
      new Translation2d(driveXDirCmd, driveYDirCmd),
      driveZRotCmd,
      driveFieldOriented,
      false,
      driveRotateLeftPtCmd,
      driveRotateRightPtCmd);

    /*
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
    */
  }

  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
  public Pose2d getPose() {
    return driveTrain.getPose();
  }

  /**
  * Returns the currently heading robot.
  *
  * @return The heading.
  */
  public Rotation2d getHeading() {
    return driveTrain.getHeading();
  }
  
  /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    driveTrain.resetOdometry(pose);
  }
    
  /**
  * Resets the Gyro angle to specified heading
  *
  */
  public void zeroGyro() {
    gyroSubSys.zeroYaw();
  }

  /**
  * Returns Drive SubSystem Kinematics
  *
  * @return Drive SubSystem Kinematics
  */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return driveTrain.getSwerveDriveKinematics();
  }

  /**
   * Returns Max Drive SubSystem Speed
   * 
   * @return double DriveTrain Maximum Speed (m/s)
   */
  public double getMaxDriveSubSysSpd(){
    return RobotSettings.DriveTrain.DriveTrainMaxSpd;
  }

  /**
   * Returns Max Drive Subsystem Rotation
   * @return double DriveTrain Maximum Speed (rads/s)
   */
  public double getMaxDriveSubSysRotSpd(){
    return RobotSettings.DriveTrain.MaxDriveSubSysRotSpeed;
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
    boolean rotateLeftPtCmd,
    boolean rotateRightPtCmd) {
      
    //Limit Cmds to Chassis Limits
    driveXDirCmd = Math.min(Math.max(xSpdCmd,-RobotSettings.DriveTrain.DriveTrainMaxSpd),RobotSettings.DriveTrain.DriveTrainMaxSpd);
    driveYDirCmd = Math.min(Math.max(ySpdCmd,-RobotSettings.DriveTrain.DriveTrainMaxSpd),RobotSettings.DriveTrain.DriveTrainMaxSpd);
    driveZRotCmd = Math.min(Math.max(rotSpdCmd,-RobotSettings.DriveTrain.MaxDriveSubSysRotSpeed),RobotSettings.DriveTrain.MaxDriveSubSysRotSpeed);
    driveFieldOriented = fieldRelative;
    driveRotateLeftPtCmd = rotateLeftPtCmd;
    driveRotateRightPtCmd = rotateRightPtCmd;
  }
}
