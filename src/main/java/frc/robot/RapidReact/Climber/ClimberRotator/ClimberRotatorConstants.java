// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberRotator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/** Add your docs here. */
public final class ClimberRotatorConstants { 
  
  public static final class ClimberRotatorMtr {
    
    public static final TalonFXInvertType kInverted = 
      TalonFXInvertType.Clockwise;
    
    public static final NeutralMode kNeutralMode = 
      NeutralMode.Brake;

    public static final double kNeutralDeadBand = 0.04;
    public static final int kNominalOutputFwd = 0;
    public static final int kNominalOutputRev = 0;
    public static final int kPeakOutputFwd = 1;
    public static final int kPeakOutputRev = -1;
    public static final double kOpenloopRamp = 1;
    public static final double kClosedloopRamp = 0;

    // Soft Limits
    public static final boolean kFwdSoftLimitEnable = false;
    public static final double kFwdSoftLimitThd = 100000;
    public static final boolean kRevSoftLimitEnable = false;
    public static final double kRevSoftLimitThd = -100000;

    public static final SensorInitializationStrategy kSensorInitStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    // FFMax (% Output with arm horizontal)
    public static final double kFFMax = 1.0;  // % Output

    public final class PosCtrlFdbk{
      
      // Slot 0       
      public static final int kPIDSlotIdx = 0; 
      public static final int kTimeoutMs = 0;
    
      // PID Settings
      public static final double kPID_P = 5.5;   
      public static final double kPID_I = 1.5;
      public static final double kPID_D = 0;
      public static final double kPID_FF = 0; 
      public static final double kPIDIMax = 50;
      public static final double KIntegralZone = 50;

      // MotionMagic Settings
      public static final double kMotionCruiseVelocity = 50;  // Units/100ms   (1/10th of desired velocity)
      public static final double kMotionAcceleration = 40;   // Units/100ms/s
      public static final int kSCurveStrength = 0;         // 0 Trapazoid 1-8 S curve

      // Allowable Closed Loop Error
      public static final double kClosedLoopPosError = 2.5;  // degrees
      public static final double kClosedLoopPosMaxVel = 10/10; // degrees/100ms
    }

    /*
    public final class VelCtrlFdbk{

      //public final TalonFXFeedbackDevice kFeedbackDevice = TalonFXFeedbackDevice.IntegratedSensor;
         
      public static final int kPIDSlotIdx = 1; 
      public static final int kTimeoutMs = 0;
    
      public static final double kPID_P = 0.05;
      public static final double kPID_I = 0;
      public static final double kPID_D = 0;
      public static final double kPID_FF = 0;
      public static final double kPIDIMax = 200;
    }
    */
  }
  public final class ClimberRotatorInternalEncoder {

    //public static final double kGearBoxRatio = 1.0/9.0;
    // Ratio is #OutputTurns/#InputTurns
  
    //public static final double kBeltRatio = 18.0/60.0;
    // Ratio is #InputTeeth/#OutputTeeth

    //public static final double kFeedbackCoefficient = 1;
    public static final double kFeedbackCoefficient = 0.00292933;  // New Gear 3/11 
    //public static final double kFeedbackCoefficient = 0.00325438; 
    //public static final double kFeedbackCoefficient = 0.001953867; 
    // 360 of Arm Rotation/65,536 Raw Cnts (2^16)

    // Initial Sensor Position
    public static final double kInitialSensorPos = -30724;
    //Math.round(-90.0/0.00292933);  // Raw Sensor Units
    // Desired Pos / kFeedbackCoefficient
  }

  public final class ClimberRotatorExternalEncoder {

    //public static final double kGearBoxRatio = 1.0/9.0;
    // Ratio is #OutputTurns/#InputTurns
  
    //public static final double kBeltRatio = 18.0/60.0;
    // Ratio is #InputTeeth/#OutputTeeth

    // Initial Sensor Position
    public static final double kInitialSensorPos = -90;

    public static final double kFeedbackCoefficient = 0.001953867; 
    // 360 of Arm Rotation/65,536 Raw Cnts (2^16)
  }

}
