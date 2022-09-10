/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;

/**
 * Add your docs here.
 */
public class SwerveModule_Type_1_Constants {
    
    public static final class FL_Module {

        public static final SwerveModule_Type_1_ModuleConstants 
        FL_Module_Settings = new SwerveModule_Type_1_ModuleConstants(
            "FL",
            true,
            false,
            false,
            false,
            887,
            true);
    }

    public static final class FR_Module {
    
        public static final SwerveModule_Type_1_ModuleConstants 
        FR_Module_Settings = new SwerveModule_Type_1_ModuleConstants(
            "FR",
            false,
            false,
            false,
            false,
            711,
            false);       
    } 

    public static final class RL_Module {
    
        public static final SwerveModule_Type_1_ModuleConstants 
        RL_Module_Settings = new SwerveModule_Type_1_ModuleConstants(
            "RL",
            true,
            false,
            false,
            false,
            201,
            false);    
    }

    public static final class RR_Module {
    
        public static final SwerveModule_Type_1_ModuleConstants 
        RR_Module_Settings = new SwerveModule_Type_1_ModuleConstants(
            "RR",
            false,
            false,
            false,
            false,
            731,
            false); 
    }
            
    public static final class SwerveModuleConstants {
    
        // Swerve Command Optimization
        public static final boolean OptimizedSwerveCmd = true;
    }   

    public static final class DriveMotorConstants {
    
        // Drive Motor Settings (Falcon TalonSRX)
        public static final NeutralMode NMode = NeutralMode.Coast;
        // *** Motor Feedback Device
        public static final FeedbackDevice FbkDevice = FeedbackDevice.IntegratedSensor;
        public static final SensorInitializationStrategy SensorInitStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        public static final boolean FeedbackNotContinuous = false;
        public static final double FeedbackCoefficient = 1.0;

        // *** Motor Feedback Counts Per Motor Revolution
        public static final double FbkEncoderCntsPerRev = 2048;
            
        // *** Motor Max Speed 
        public static final double MaxSpeed = 6380;
                
        // *** Motor Controller
        public static final int ControllerID = 0;
        public static final double P = 0.2;  //0.035
        public static final double I = 0.3;  //0 001
        public static final double D = 0;
        public static final double AllowableClosedLoopError = 0.01;     // m/s
        public static final double AllowableClosedLoopDerError = 0.001;  // m/s^2
        public static final double ClosedloopRamp = 0;  // Seconds to full
        public static final double FF_gain = 1/4.1148;  // OutputFraction/(max m/s)
        // max m/s = ((MaxSpeed/GearRatio)*(pi*WheelDiameter))/60  - RPM to m/s
        // max m/s = ((6380/6.67)*(pi*0.1016))/60 = 5.088
        // Andy Mark says 13.5ft/s = 4.1148

    }
            
    // Drive Module Constants
    public static final class DriveModuleConstants {
        
        // *** Drive Module Gear Ratio
        public static final double GearRatio = 6.67; 
        
        // *** Drive Module Wheel Diameter
        public static final double WheelDiameter = Units.inchesToMeters(4.0);

        // *** Drive Module Max Wheel Speed
        public static final double MaxDriveWheelSpd = 5.0;
    }
    
    public static final class SteerMotorConstants {
    
        // Steer Motor Settings (Talon SRX)

        // *** Motor Feedback Device
        public static final FeedbackDevice FdkDevice = FeedbackDevice.Analog;
        public static final double FeedbackGain = 1;
        public static final boolean ConifgFeedbackNotContinuous = true;

        // *** Motor Feedback Counts Per Motor Revolution
        public static final int SensorCntsPerRev = 1024;
        public static final double SensorCnts2Rads = ((2*Math.PI)/SensorCntsPerRev);
        public static final double SensorRads2Cnts = (SensorCntsPerRev/(2*Math.PI));

        // *** Motor Max Speed 

        // *** Motor Controller
        public static final int ControllerID = 0;
        public static final double P = 0.5;
        public static final double I = 1.0;
        public static final double D = 0.0;
        public static final double MinContinuousInput = -Math.PI;
        public static final double MaxContinouseInput = Math.PI;
        public static final double AllowableClosedLoopError = 0.01;    // Rads
        public static final double AllowableClosedLoopDerError = 0.001;  // Rads/s
        public static final double MinIntegral = -0.045;
        public static final double MaxIntegral = 0.045;
        public static final double ClosedloopRamp = 0;  // Seconds to full
    }   
}
