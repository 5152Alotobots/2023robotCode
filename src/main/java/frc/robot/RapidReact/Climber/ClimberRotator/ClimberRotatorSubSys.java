// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberRotator;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RapidReact.Climber.ClimberInteraction;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorConstants.ClimberRotatorInternalEncoder;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorConstants.ClimberRotatorMtr;

public class ClimberRotatorSubSys extends SubsystemBase {
  /** Creates a new Climber_RotatorSubSys. */

  private TalonFX m_ClimberRotatorMtr;
  private AnalogInput m_ClimberRotatorEnc;
  private ClimberInteraction m_ClimberInteraction;

  public ClimberRotatorSubSys(ClimberInteraction climberInteraction) {
    m_ClimberInteraction = climberInteraction;

    // Initialize Climber Rotator External Encoder
    m_ClimberRotatorEnc = new AnalogInput(Constants.AnalogInput_IDs.ClimberRotatorEnc_ID);

    // Initialize Climber Rotator Motor
    m_ClimberRotatorMtr = new TalonFX(Constants.CAN_IDs.ClimberRotatorMtr_ID);
    m_ClimberRotatorMtr.configFactoryDefault();
    m_ClimberRotatorMtr.setInverted(ClimberRotatorMtr.kInverted);
    m_ClimberRotatorMtr.setNeutralMode(ClimberRotatorMtr.kNeutralMode);

 		// Config the peak and nominal outputs
    m_ClimberRotatorMtr.configNominalOutputForward(ClimberRotatorMtr.kNominalOutputFwd);
    m_ClimberRotatorMtr.configNominalOutputReverse(ClimberRotatorMtr.kNominalOutputRev);
    m_ClimberRotatorMtr.configPeakOutputForward(ClimberRotatorMtr.kPeakOutputFwd);
    m_ClimberRotatorMtr.configPeakOutputReverse(ClimberRotatorMtr.kPeakOutputRev);
    m_ClimberRotatorMtr.configNeutralDeadband(ClimberRotatorMtr.kNeutralDeadBand);
    
    m_ClimberRotatorMtr.configOpenloopRamp(ClimberRotatorMtr.kOpenloopRamp);
    m_ClimberRotatorMtr.configClosedloopRamp(ClimberRotatorMtr.kClosedloopRamp, 0);

    // Config Interal Encoder
    m_ClimberRotatorMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    m_ClimberRotatorMtr.configSelectedFeedbackCoefficient(ClimberRotatorInternalEncoder.kFeedbackCoefficient);
    m_ClimberRotatorMtr.setSelectedSensorPosition(ClimberRotatorInternalEncoder.kInitialSensorPos);

    // Set Soft Position Limits
    m_ClimberRotatorMtr.configForwardSoftLimitEnable(ClimberRotatorMtr.kFwdSoftLimitEnable);
    m_ClimberRotatorMtr.configForwardSoftLimitThreshold(ClimberRotatorMtr.kFwdSoftLimitThd);
    m_ClimberRotatorMtr.configReverseSoftLimitEnable(ClimberRotatorMtr.kRevSoftLimitEnable);
    m_ClimberRotatorMtr.configReverseSoftLimitThreshold(ClimberRotatorMtr.kRevSoftLimitThd);

    // 0
    // Config Position Closed Loop
    m_ClimberRotatorMtr.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx, 
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);

    m_ClimberRotatorMtr.setSelectedSensorPosition(
      ClimberRotatorInternalEncoder.kInitialSensorPos,
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx, 
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);

    m_ClimberRotatorMtr.config_kP(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.kPID_P);
    
    m_ClimberRotatorMtr.config_kI(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.kPID_I);
    
    m_ClimberRotatorMtr.config_kD(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.kPID_D);
    
    m_ClimberRotatorMtr.config_kF(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.kPID_FF);

    m_ClimberRotatorMtr.configMaxIntegralAccumulator(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.kPIDIMax);

    m_ClimberRotatorMtr.config_IntegralZone(
      ClimberRotatorMtr.PosCtrlFdbk.kPIDSlotIdx,
      ClimberRotatorMtr.PosCtrlFdbk.KIntegralZone,
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);
  
    m_ClimberRotatorMtr.configMotionCruiseVelocity(
      ClimberRotatorMtr.PosCtrlFdbk.kMotionCruiseVelocity,
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);
  
    m_ClimberRotatorMtr.configMotionAcceleration(
      ClimberRotatorMtr.PosCtrlFdbk.kMotionAcceleration,
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);
  
    m_ClimberRotatorMtr.configMotionSCurveStrength(
      ClimberRotatorMtr.PosCtrlFdbk.kSCurveStrength, 
      ClimberRotatorMtr.PosCtrlFdbk.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberRotatorPos", m_ClimberRotatorMtr.getSelectedSensorPosition());

    // Update Climber Interaction
    m_ClimberInteraction.setRotatorPos(m_ClimberRotatorMtr.getSelectedSensorPosition());
    updateLiftSoftLimits();

    SmartDashboard.putNumber("ClimberRotator_LiftPos", m_ClimberInteraction.getLiftPos());

    //SmartDashboard.putBoolean("ClimberRotatorAtPos", isClimberRotatorAtPosCmd());
    SmartDashboard.putNumber("ClimberPower",m_ClimberRotatorMtr.getMotorOutputPercent());
  }

  public void setClimberRotatorSpdCmd(double rotatorSpdCmd, TalonFXControlMode ctrlMode){
    m_ClimberRotatorMtr.set(ctrlMode, rotatorSpdCmd);
    // Was set to SpdCmd*20000
  }

  public void setClimberRotatorPosCmd(double rotatorPosCmd, TalonFXControlMode ctrlMode, boolean enableFF, double angleOffset){
    double rotPosRads = 0.0;
    double rotPosRadsOffset = 0.0;
    double cosRotPos = 0.0;
    double ffCmd = 0.0;
    if (enableFF){
      // Convert to Radians
      rotPosRads = Units.degreesToRadians(m_ClimberRotatorMtr.getSelectedSensorPosition());
      SmartDashboard.putNumber("RotPosRads", rotPosRads);
      
      // Add Offset to compensate for traversal
      rotPosRadsOffset = rotPosRads + Units.degreesToRadians(angleOffset);
      SmartDashboard.putNumber("RotPosRadsOffset", rotPosRadsOffset);

      // Calculate Cos
      cosRotPos = Math.cos(rotPosRadsOffset);
      SmartDashboard.putNumber("CosRotPos", cosRotPos);

      // Calculate FF
      //ffCmd = Math.max(0, cosRotPos*ClimberRotatorMtr.kFFMax);
      ffCmd = Math.abs(cosRotPos*ClimberRotatorMtr.kFFMax);
    }
    SmartDashboard.putNumber("FFCmd", ffCmd);
    
    m_ClimberRotatorMtr.set(ctrlMode, rotatorPosCmd, DemandType.ArbitraryFeedForward, ffCmd);
    SmartDashboard.putNumber("RotatorPercent", m_ClimberRotatorMtr.getMotorOutputPercent());  
  }

  public void setClimberRotatorPos(double rotatorEncoderPos){
    m_ClimberRotatorMtr.setSelectedSensorPosition(rotatorEncoderPos);
  }

  public double getRotatorPos(){
    return m_ClimberRotatorMtr.getSelectedSensorPosition();
  }

  public boolean isClimberRotatorAtPosCmd(){
    
    boolean posCheck = Math.abs(m_ClimberRotatorMtr.getClosedLoopError(0)) < ClimberRotatorMtr.PosCtrlFdbk.kClosedLoopPosError;
    boolean velCheck = Math.abs(m_ClimberRotatorMtr.getSelectedSensorVelocity(0)) < ClimberRotatorMtr.PosCtrlFdbk.kClosedLoopPosMaxVel;
    
    double posTarget = 0.0;
    posTarget = m_ClimberRotatorMtr.getClosedLoopTarget(0);
    SmartDashboard.putNumber("ClimberRotatorPosTarget", posTarget);
    SmartDashboard.putNumber("ClimberRotatorPosError", m_ClimberRotatorMtr.getClosedLoopError(0));
    
    double posCheckDbl = 0.0;
    double velCheckDbl = 0.0;

    if (posCheck){
      posCheckDbl = 1.0;
    } else {
      posCheckDbl = 0.0;
    }

    if (velCheck){
      velCheckDbl = 1.0;
    } else {
      velCheckDbl = 0.0;
    }

    SmartDashboard.putNumber("ClimberRotatorPosCheck", posCheckDbl);
    SmartDashboard.putNumber("ClimberRotatorVelCheck", velCheckDbl);

    boolean rotatorAtPos = false;
    double rotatorAtPosDbl = 0.0;
    
    if (posCheck && velCheck){
      rotatorAtPos = true;
      rotatorAtPosDbl = 1.0;
    } else {
      rotatorAtPos = false;
      rotatorAtPosDbl = 0.0;
    }

    SmartDashboard.putNumber("ClimberRotatorAtPos",rotatorAtPosDbl);
    return rotatorAtPos; 
  }

  public boolean isClimberRotatorAtPosCmdCustom(double maxPosError, double maxVelError){
    
    boolean posCheck = Math.abs(m_ClimberRotatorMtr.getClosedLoopError(0)) < maxPosError;
    boolean velCheck = Math.abs(m_ClimberRotatorMtr.getSelectedSensorVelocity(0)) < maxVelError;
    
    SmartDashboard.putNumber("ClimberRotatorPosTarget", m_ClimberRotatorMtr.getClosedLoopTarget(0));
    SmartDashboard.putNumber("ClimberRotatorPosError", m_ClimberRotatorMtr.getClosedLoopError(0));
    
    double posCheckDbl = 0.0;
    double velCheckDbl = 0.0;

    if (posCheck){
      posCheckDbl = 1.0;
    } else {
      posCheckDbl = 0.0;
    }

    if (velCheck){
      velCheckDbl = 1.0;
    } else {
      velCheckDbl = 0.0;
    }

    SmartDashboard.putNumber("ClimberRotatorPosCheck", posCheckDbl);
    SmartDashboard.putNumber("ClimberRotatorVelCheck", velCheckDbl);

    boolean rotatorAtPos = false;
    double rotatorAtPosDbl = 0.0;
    
    if (posCheck && velCheck){
      rotatorAtPos = true;
      rotatorAtPosDbl = 1.0;
    } else {
      rotatorAtPos = false;
      rotatorAtPosDbl = 0.0;
    }

    SmartDashboard.putNumber("ClimberRotatorAtPos",rotatorAtPosDbl);
    return rotatorAtPos; 
  }

  private void updateLiftSoftLimits(){
    double fwdTravelLimit = m_ClimberInteraction.getRotatorMaxPos(); 
    double revTravelLimit = m_ClimberInteraction.getRotatorMinPos();
    m_ClimberRotatorMtr.configForwardSoftLimitThreshold(fwdTravelLimit);
    m_ClimberRotatorMtr.configReverseSoftLimitThreshold(revTravelLimit);
    SmartDashboard.putNumber("ClimberRotatorFwdLimit", fwdTravelLimit);
    SmartDashboard.putNumber("ClimberRotatorRevLimit", revTravelLimit);
  }
}
