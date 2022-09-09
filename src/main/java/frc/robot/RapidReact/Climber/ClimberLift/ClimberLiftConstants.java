// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberLift;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/** Add your docs here. */
public class ClimberLiftConstants {
  
  public static final class ClimberLiftMtr {
    
    public static final TalonFXInvertType kInverted = 
      TalonFXInvertType.CounterClockwise;
    
    // Temp
    public static final NeutralMode kNeutralMode = 
      NeutralMode.Brake;

    //public static final NeutralMode kNeutralMode = 
    //  NeutralMode.Brake;

    public static final double kNeutralDeadBand = 0.04;
    public static final int kNominalOutputFwd = 0;
    public static final int kNominalOutputRev = 0;
    public static final int kPeakOutputFwd = 1;
    public static final int kPeakOutputRev = -1;
    public static final double kOpenloopRamp = 1;
    public static final double kClosedloopRamp = 1;

    // Soft Limits
    public static final boolean kFwdSoftLimitEnable = false;
    public static final double kFwdSoftLimitThd = ClimberLiftEncoder.kMaxClimberLiftPos;
    public static final boolean kRevSoftLimitEnable = false;
    public static final double kRevSoftLimitThd = ClimberLiftEncoder.kMinClimberLiftPos;
    
    public static final SensorInitializationStrategy kSensorInitStrategy = SensorInitializationStrategy.BootToZero;

    // Slot 0 - No Load Vertical
    public final class Slot0 {
      
      // Slot
      public static final int kPIDSlotIdx = 0; 
      public static final int kTimeoutMs = 0;
    
      // PID Settings
      public static final double kPID_P = 8.0;  // 0.75
      public static final double kPID_I = 1.0;
      public static final double kPID_D = 0.0;  // 4.5
      public static final double kPID_FF = 0; 
      public static final double kPIDIMax = 40;
      public static final double KIntegralZone = 10;

      // MotionMagic Settings
      public static final double kMotionCruiseVelocity = 200; //100 // Units/100ms   (1/10th of desired velocity)
      public static final double kMotionAcceleration = 1000;   // Units/100ms/s
      public static final int kSCurveStrength = 8;         // 0 Trapazoid 1-8 S curve

      // Allowable Closed Loop Error
      public static final double kClosedLoopPosError = 2.5;  // mm
      public static final double kClosedLoopPosMaxVel = 100/10; // degrees/100ms

      // Max Output
      public static final double kClosedLoopPeakOutput = 0.7; // Percent Output
    }

    // Slot 1 - Full Load Vertical
    public final class Slot1{

      //public final TalonFXFeedbackDevice kFeedbackDevice = TalonFXFeedbackDevice.IntegratedSensor;
         
      public static final int kPIDSlotIdx = 1; 
      public static final int kTimeoutMs = 0;
    
      public static final double kPID_P = 8.0;
      public static final double kPID_I = 1.0;
      public static final double kPID_D = 0.0;
      public static final double kPID_FF = -0.15;
      public static final double kPIDIMax = 40;
      public static final double KIntegralZone = 25;

      // MotionMagic Settings
      public static final double kMotionCruiseVelocity = 550; //200; //100 // Units/100ms   (1/10th of desired velocity)
      public static final double kMotionAcceleration = 1000;   // Units/100ms/s
      public static final int kSCurveStrength = 8;         // 0 Trapazoid 1-8 S curve
      
      // Allowable Closed Loop Error
      public static final double kClosedLoopPosError = 2.5;  // m
      public static final double kClosedLoopPosMaxVel = 100/10; // mm/100ms

      // Max Output
      public static final double kClosedLoopPeakOutput = 0.9;// Percent Output 
    }
  
    // Slot 2 - Full Load Angled
    public final class Slot2{

      //public final TalonFXFeedbackDevice kFeedbackDevice = TalonFXFeedbackDevice.IntegratedSensor;
       
      public static final int kPIDSlotIdx = 2; 
      public static final int kTimeoutMs = 0;
  
      public static final double kPID_P = 8.0;
      public static final double kPID_I = 1.0;
      public static final double kPID_D = 0.0;
      public static final double kPID_FF = 0.0;
      public static final double kPIDIMax = 40;
      public static final double KIntegralZone = 25;

      // MotionMagic Settings
      public static final double kMotionCruiseVelocity = 550;//200; //100 // Units/100ms   (1/10th of desired velocity)
      public static final double kMotionAcceleration = 1000;   // Units/100ms/s
      public static final int kSCurveStrength = 8;         // 0 Trapazoid 1-8 S curve
    
      // Allowable Closed Loop Error
      public static final double kClosedLoopPosError = 2.5;  // mm
      public static final double kClosedLoopPosMaxVel = 100/10; // mm/100ms

      // Max Output
      public static final double kClosedLoopPeakOutput = 0.9; // Percent Output
    }
  }
  
  public final class ClimberLiftEncoder {
    public static final boolean kSensorPhase = true;

    public static final double kMinClimberLiftPosRawSensor = 0;
    public static final double kMinClimberLiftPos = 2;
    // 0 inches of travel
    public static final double kMaxClimberLiftPosRawSensor = 20706;  // cnts
    public static final double kMaxClimberLiftPos = 513;  // mm
    // 515mm of travel

    // Initial Sensor Position
    public static final double kInitialSensorPos = 0;

    public static final double kFeedbackCoefficient = 515.0/20706.0; //0.09249064; //1; //(1.0/4096.0)*kRotatorRatio;
    // 515mm/20706 cnts
    // 1Rev/2^12units*21000ThousandthsInches/5.5Rev
    // 2^12 = 4096
  }
}