// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeInNOut;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class IntakeInNOutConstants {

  public static final class IntakeInNOutMtr {
    public static final TalonFXInvertType kInverted = 
      TalonFXInvertType.CounterClockwise;
    
    public static final NeutralMode kNeutralMode = 
      NeutralMode.Coast;
    
    public static final double kVoltageCompSaturation = 10.5;  // Volts
    public static final boolean kEnableVoltageComp = true;

    public static final TalonFXFeedbackDevice kFeedbackDevice = 
      TalonFXFeedbackDevice.IntegratedSensor;
    public static SensorInitializationStrategy kSensorInitStrategy = SensorInitializationStrategy.BootToZero;
    
    public static double kEncoderRes = 2048.0;
    public static double kMotorGearTeeth = 16.0;
    public static double kWheelGearTeeth = 24.0;
    public static double kWheelDiameter = Units.inchesToMeters(4.0);  // 0.1016m = 4inches
    public static double kFeedbackCoefficient = 1; /*
      (2*Math.PI/kEncoderRes)*(kMotorGearTeeth/kWheelGearTeeth)*
      ((Math.PI*kWheelDiameter)/(2*Math.PI));  
*/
    public static final int kPIDSlotIdx = 0; 
    public static final int kTimeoutMs = 0;
    
    public static final double kPID_P = 0.06;
    public static final double kPID_I = 0.0001;
    public static final double kPID_D = 0;
    public static final double kPID_FF = 0.048;
    public static final double kAllowableError = 10;
    public static final double kIZone = 1000;
    public static final double kMaxIAccum = 0.5;

    public static final double kNeutralDeadBand = 0.1;
    public static final int kNominalOutputFwd = 0;
    public static final int kNominalOutputRev = 0;
    public static final int kPeakOutputFwd = 1;
    public static final int kPeakOutputRev = -1;

    public static final double kOpenloopRamp = 1;
    public static final double kClosedloopRamp = 1;
  }

  public static final class IntakeInNOutLwrMtr {
    
    public static final TalonFXInvertType kInverted = 
      TalonFXInvertType.CounterClockwise;
    
    public static final NeutralMode kNeutralMode = 
      NeutralMode.Coast;
    
    public static final double kVoltageCompSaturation = 10.5;  // Volts
    public static final boolean kEnableVoltageComp = true;

    public static final TalonFXFeedbackDevice kFeedbackDevice = 
      TalonFXFeedbackDevice.IntegratedSensor;
    public static SensorInitializationStrategy kSensorInitStrategy = SensorInitializationStrategy.BootToZero;
    
    // Feedback Coefficient
    public static double kEncoderRes = 2048.0;
    public static double kMotorGearTeeth = 16.0;
    public static double kWheelGearTeeth = 24.0;
    public static double kWheelDiameter = Units.inchesToMeters(0.825); // 4inches
    public static double kFeedbackCoefficient = 1; /*
      (2*Math.PI/kEncoderRes)*(kMotorGearTeeth/kWheelGearTeeth)*
      ((Math.PI*kWheelDiameter)/(2*Math.PI));   
*/
    public static final int kPIDSlotIdx = 0; 
    public static final int kTimeoutMs = 0;
    
    public static final double kPID_P = 0.05;
    public static final double kPID_I = 0.00001;
    public static final double kPID_D = 0;
    public static final double kPID_FF = 0.05;
    public static final double kAllowableError = 100;
    public static final double kIZone = 10000;
    public static final double kMaxIAccum = 0.5;

    public static final double kNeutralDeadBand = 0.1;
    public static final int kNominalOutputFwd = 0;
    public static final int kNominalOutputRev = 0;
    public static final int kPeakOutputFwd = 1;
    public static final int kPeakOutputRev = -1;

    public static final double kOpenloopRamp = 1;
    public static final double kClosedloopRamp = 1;
  }
}
