/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Add your docs here.
 */
public class SwerveModule_Type_3_Constants {
    
    public static final class DriveConstants {
        
        public static final class DriveMotorConstants {
    
            // Drive Motor Settings (Spark Max w/ Neos)
            // *** CAN Addresses ***  
            public static final int kFrontLeftPort = 4;
            public static final int kRearLeftPort = 6;
            public static final int kFrontRightPort = 2;
            public static final int kRearRightPort = 8;
        
            // *** Motor Direction Inverted
            public static final boolean kFrontLeftInverted = false;
            public static final boolean kRearLeftInverted = false;
            public static final boolean kFrontRightInverted = false;
            public static final boolean kRearRightInverted = false;
        
            // *** Motor Max Speed 
            public static final double kMaxSpeed = 5700;
            
            // *** Motor Controller
            public static final int kPID_ID = 0;
            public static final double kP = 5e-5;
            public static final double kI = 1e-6;
            public static final double kD = 0;
        }
        
        // Drive Module Contsants
        public static final class DriveModuleConstants {
            // *** Drive Module Gear Ratio
            public static final double kGearRatio = 6.67; 
            // *** Drive Module Wheel Diameter
            public static final double kWheelDiameter = Units.inchesToMeters(4.0);
        }

        public static final class SteerMotorConstants {
            // Steer Motor Settings (Talon SRX)
            // *** CAN Addresses ***
            public static final int kFrontLeftPort = 3;
            public static final int kRearLeftPort = 5;
            public static final int kFrontRightPort = 1;
            public static final int kRearRightPort = 7;
    
            // *** Motor Direction Inverted ***
            public static final boolean kFrontLeftInverted = false;
            public static final boolean kRearLeftInverted = false;
            public static final boolean kFrontRightInverted = false;
            public static final boolean kRearRightInverted = false;
        
            public static final FeedbackDevice kFeedbackDevice = FeedbackDevice.Analog;
            public static final double kFeedbackGain = 1;
            public static final boolean kconifgFeedbackNotContinuous = false;
            public static final int kconfigFeedbackNotContinuousTimeOut = 20;
            public static final boolean ksetSensorPhase = false;
            public static final int kControllerID = 0;
            public static final double kP = 10;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kSensorCnts2Rads = ((2*Math.PI)/1024);
            public static final double kSensorRads2Cnts = (1024/(2*Math.PI));
        }
  
        public static final double kTrackWidth = 0.5;
        //Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        //Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        public static final boolean kGyroReversed = true;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    
        public static final double kMaxSpeedMetersPerSecond = 3;
    }
}
