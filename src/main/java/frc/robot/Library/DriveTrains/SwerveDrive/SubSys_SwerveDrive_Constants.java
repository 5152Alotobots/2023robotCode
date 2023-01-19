// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.*;

/** Add your docs here. */
public class SubSys_SwerveDrive_Constants {

    /* Swerve Drive Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);

    /** Swerve Kinematics
     * 
     * (0,0) is at the center of the Robot 
     * 
     * +x => Forward
     * +y => Left
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),       // FL - Front Left
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),      // FR - Front Right
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),      // BL - Back Left
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));    // BR - Back Right

    /** Swerve Module Constants */
    public static final SwerveModuleConstants FL_constants = new SwerveModuleConstants(
        "FL",                                       // Module Name
        Constants.CAN_IDs.FrontLeftDriveMtr_ID,     // Drive Motor CAN ID
        false,                                      // Drive Motor Inverted
        Constants.CAN_IDs.FrontLeftSteerMtr_ID,     // Steer Motor CAN ID
        false,                                      // Steer Motor Inverted   
        255,                                        // CANCoder ID (255 for not used)       
        10.0);                                      // Degrees

    public static final SwerveModuleConstants FR_constants = new SwerveModuleConstants(
        "FR",                                       // Module Name
        Constants.CAN_IDs.FrontRightDriveMtr_ID,    // Drive Motor CAN ID
        false,                                      // Drive Motor Inverted
        Constants.CAN_IDs.FrontRightSteerMtr_ID,    // Steer Motor CAN ID
        false,                                      // Steer Motor Inverted   
        255,                                        // CANCoder ID (255 for not used)       
        10.0);                                      // Degrees 

    public static final SwerveModuleConstants BL_constants = new SwerveModuleConstants(
        "BL",                                       // Module Name
        Constants.CAN_IDs.BackLeftDriveMtr_ID,      // Drive Motor CAN ID
        false,                                      // Drive Motor Inverted
        Constants.CAN_IDs.BackLeftSteerMtr_ID,      // Steer Motor CAN ID
        false,                                      // Steer Motor Inverted   
        255,                                        // CANCoder ID (255 for not used)       
        10.0);                                      // Degrees

    public static final SwerveModuleConstants BR_constants = new SwerveModuleConstants(
        "BR",                                       // Module Name
        Constants.CAN_IDs.BackRightDriveMtr_ID,     // Drive Motor CAN ID
        false,                                      // Drive Motor Inverted
        Constants.CAN_IDs.BackRightSteerMtr_ID,     // Steer Motor CAN ID
        false,                                      // Steer Motor Inverted   
        255,                                        // CANCoder ID (255 for not used)       
        10.0);                                      // Degrees

    public static final Translation2d RotationPtFL = new Translation2d(
        0.6096,
        trackWidth / 2);  
        
    public static final Translation2d RotationPtFR = new Translation2d(
        0.6096,
        -trackWidth / 2);

    public static final Translation2d RotationPtBL = new Translation2d(
        -wheelBase / 2,
        trackWidth / 2);
    
    public static final Translation2d RotationPtBR = new Translation2d(
        -wheelBase / 2,
        -trackWidth / 2);
    
    public static final double RotateFieldRelativeMargin = 10; // Degrees
    
}
