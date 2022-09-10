/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Add your docs here.
 */
public class SwerveDriveSubSys_Constants {

    // SwerveDrive Subsystem Enable Shuffleboard
    public static final boolean SwerveDriveSubSys_Shuffleboard_Enable = false; 

    //Distance between centers of right and left wheels on robot
    public static final double TrackWidth = 0.508;

    //Distance between front and back wheels on robot
    public static final double WheelBase = 0.5842;
    
    public static final SwerveDriveKinematics DriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(WheelBase / 2, TrackWidth / 2),
        new Translation2d(WheelBase / 2, -TrackWidth / 2),
        new Translation2d(-WheelBase / 2, TrackWidth / 2),
        new Translation2d(-WheelBase / 2, -TrackWidth / 2));
    
    public static final Translation2d RotationPtFL = new Translation2d(
      0.6096,
      SwerveDriveSubSys_Constants.TrackWidth / 2);  
      
    public static final Translation2d RotationPtRL = new Translation2d(
      -SwerveDriveSubSys_Constants.WheelBase / 2,
      SwerveDriveSubSys_Constants.TrackWidth / 2);
  
    public static final Translation2d RotationPtRR = new Translation2d(
      -SwerveDriveSubSys_Constants.WheelBase / 2,
      -SwerveDriveSubSys_Constants.TrackWidth / 2);
  
    public static final Translation2d RotationPtFR = new Translation2d(
      0.6096,
      -SwerveDriveSubSys_Constants.TrackWidth / 2);

    public static final double RotateFieldRativeMargin = Math.PI/16; 

    // Trajectory Following PID's
    public static final double PGain_Translation = 0.5;
    public static final double IGain_Translation = 0;
    public static final double DGain_Translation = 0;

    public static final double PGain_Rotation = 1.0;
    public static final double IGain_Rotation = 0;
    public static final double DGain_Rotation = 0;

}
