// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberLift;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RapidReact.Climber.ClimberInteraction;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftConstants.ClimberLiftEncoder;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftConstants.ClimberLiftMtr;

public class ClimberLiftSubSys extends SubsystemBase {
  /** Creates a new ClimberLiftSubSys. */

  private TalonFX m_ClimberLiftMtr;
  private CANCoder m_ClimberLiftEncoder;
  private ClimberInteraction m_ClimberInteraction;
  private double m_LiftRotatorOffset;
  private int m_ClimberLiftActiveSlot = ClimberLiftMtr.Slot0.kPIDSlotIdx;

  public ClimberLiftSubSys(ClimberInteraction climberInteraction) {
    m_ClimberInteraction = climberInteraction;
    
    // Initialize Climber Lift Encoder
    m_ClimberLiftEncoder = new CANCoder(Constants.CAN_IDs.ClimberLiftEnc_ID);
    m_ClimberLiftEncoder.setPosition(0.0);

    // Initialize Climber Lift Motor
    m_ClimberLiftMtr = new TalonFX(Constants.CAN_IDs.ClimberLiftMtr_ID);
    m_ClimberLiftMtr.configFactoryDefault();
    m_ClimberLiftMtr.setInverted(ClimberLiftMtr.kInverted);
    m_ClimberLiftMtr.setNeutralMode(ClimberLiftMtr.kNeutralMode);

 		// Config the peak and nominal outputs
    m_ClimberLiftMtr.configNominalOutputForward(ClimberLiftMtr.kNominalOutputFwd);
    m_ClimberLiftMtr.configNominalOutputReverse(ClimberLiftMtr.kNominalOutputRev);
    m_ClimberLiftMtr.configPeakOutputForward(ClimberLiftMtr.kPeakOutputFwd);
    m_ClimberLiftMtr.configPeakOutputReverse(ClimberLiftMtr.kPeakOutputRev);
    m_ClimberLiftMtr.configNeutralDeadband(ClimberLiftMtr.kNeutralDeadBand);
       
    m_ClimberLiftMtr.configOpenloopRamp(ClimberLiftMtr.kOpenloopRamp);
    m_ClimberLiftMtr.configClosedloopRamp(ClimberLiftMtr.kClosedloopRamp, 0);

    // Config Interal Encoder
    m_ClimberLiftMtr.configIntegratedSensorInitializationStrategy(ClimberLiftMtr.kSensorInitStrategy);
    m_ClimberLiftMtr.configIntegratedSensorOffset(0.0);
      
    // Config Remote Encoder
    m_ClimberLiftMtr.configRemoteFeedbackFilter(m_ClimberLiftEncoder, 0);
    m_ClimberLiftMtr.setSensorPhase(ClimberLiftEncoder.kSensorPhase);
    m_ClimberLiftMtr.configSelectedFeedbackCoefficient(ClimberLiftEncoder.kFeedbackCoefficient,
      0, 
      0);

    // Set Soft Position Limits
    m_ClimberLiftMtr.configForwardSoftLimitEnable(ClimberLiftMtr.kFwdSoftLimitEnable);
    m_ClimberLiftMtr.configForwardSoftLimitThreshold(ClimberLiftMtr.kFwdSoftLimitThd);
    m_ClimberLiftMtr.configReverseSoftLimitEnable(ClimberLiftMtr.kRevSoftLimitEnable);
    m_ClimberLiftMtr.configReverseSoftLimitThreshold(ClimberLiftMtr.kRevSoftLimitThd);
    
    // Init Lift Rotator Offset
    m_LiftRotatorOffset = 0.0;

    // Slot 0
    m_ClimberLiftMtr.setSelectedSensorPosition(
      ClimberLiftEncoder.kInitialSensorPos,
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kTimeoutMs);
   
    // Position Closed Loop
    m_ClimberLiftMtr.configSelectedFeedbackSensor(
      RemoteFeedbackDevice.RemoteSensor0,
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kTimeoutMs);

    m_ClimberLiftMtr.config_kP(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kPID_P);
    
    m_ClimberLiftMtr.config_kI(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kPID_I);
    
    m_ClimberLiftMtr.config_kD(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kPID_D);
    
    m_ClimberLiftMtr.config_kF(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kPID_FF);

    m_ClimberLiftMtr.configMaxIntegralAccumulator(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.kPIDIMax);

    m_ClimberLiftMtr.config_IntegralZone(
      ClimberLiftMtr.Slot0.kPIDSlotIdx,
      ClimberLiftMtr.Slot0.KIntegralZone,
      ClimberLiftMtr.Slot0.kTimeoutMs);

    m_ClimberLiftMtr.configClosedLoopPeakOutput(
      ClimberLiftMtr.Slot0.kPIDSlotIdx, 
      ClimberLiftMtr.Slot0.kClosedLoopPeakOutput);

    m_ClimberLiftMtr.configMotionCruiseVelocity(
      ClimberLiftMtr.Slot0.kMotionCruiseVelocity,
      ClimberLiftMtr.Slot0.kTimeoutMs);

    m_ClimberLiftMtr.configMotionAcceleration(
      ClimberLiftMtr.Slot0.kMotionAcceleration,
      ClimberLiftMtr.Slot0.kTimeoutMs);

    m_ClimberLiftMtr.configMotionSCurveStrength(
      ClimberLiftMtr.Slot0.kSCurveStrength, 
      ClimberLiftMtr.Slot0.kTimeoutMs);

    // Slot 1
    //m_ClimberLiftMtr.setSelectedSensorPosition(
    //  ClimberLiftEncoder.kInitialSensorPos,
    //  ClimberLiftMtr.Slot1.kPIDSlotIdx,
    //  ClimberLiftMtr.Slot1.kTimeoutMs);

    //m_ClimberLiftMtr.configSelectedFeedbackCoefficient(ClimberLiftEncoder.kFeedbackCoefficient);

    // Position Closed Loop
    //m_ClimberLiftMtr.configSelectedFeedbackSensor(
    //  RemoteFeedbackDevice.RemoteSensor0,
    //  ClimberLiftMtr.Slot1.kPIDSlotIdx,
    //  ClimberLiftMtr.Slot1.kTimeoutMs);

    m_ClimberLiftMtr.config_kP(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.kPID_P);

    m_ClimberLiftMtr.config_kI(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.kPID_I);

    m_ClimberLiftMtr.config_kD(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.kPID_D);

    m_ClimberLiftMtr.config_kF(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.kPID_FF);

    m_ClimberLiftMtr.configMaxIntegralAccumulator(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.kPIDIMax);

    m_ClimberLiftMtr.config_IntegralZone(
      ClimberLiftMtr.Slot1.kPIDSlotIdx,
      ClimberLiftMtr.Slot1.KIntegralZone,
      ClimberLiftMtr.Slot1.kTimeoutMs);

    m_ClimberLiftMtr.configClosedLoopPeakOutput(
      ClimberLiftMtr.Slot1.kPIDSlotIdx, 
      ClimberLiftMtr.Slot1.kClosedLoopPeakOutput);  
      
    m_ClimberLiftMtr.configMotionCruiseVelocity(
      ClimberLiftMtr.Slot1.kMotionCruiseVelocity,
      ClimberLiftMtr.Slot1.kTimeoutMs);

    m_ClimberLiftMtr.configMotionAcceleration(
      ClimberLiftMtr.Slot1.kMotionAcceleration,
      ClimberLiftMtr.Slot1.kTimeoutMs);

    m_ClimberLiftMtr.configMotionSCurveStrength(
      ClimberLiftMtr.Slot1.kSCurveStrength, 
      ClimberLiftMtr.Slot1.kTimeoutMs);

    // Slot 2
    //m_ClimberLiftMtr.setSelectedSensorPosition(
    //  ClimberLiftEncoder.kInitialSensorPos,
    //  ClimberLiftMtr.Slot2.kPIDSlotIdx,
    //  ClimberLiftMtr.Slot2.kTimeoutMs);
    
    //m_ClimberLiftMtr.configSelectedFeedbackCoefficient(ClimberLiftEncoder.kFeedbackCoefficient);
    
    // Position Closed Loop
    //m_ClimberLiftMtr.configSelectedFeedbackSensor(
    //  RemoteFeedbackDevice.RemoteSensor0,
    //  ClimberLiftMtr.Slot2.kPIDSlotIdx,
    //  ClimberLiftMtr.Slot2.kTimeoutMs);
    
    m_ClimberLiftMtr.config_kP(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.kPID_P);
    
    m_ClimberLiftMtr.config_kI(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.kPID_I);
    
    m_ClimberLiftMtr.config_kD(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.kPID_D);
    
    m_ClimberLiftMtr.config_kF(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.kPID_FF);
    
    m_ClimberLiftMtr.configMaxIntegralAccumulator(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.kPIDIMax);
    
    m_ClimberLiftMtr.config_IntegralZone(
      ClimberLiftMtr.Slot2.kPIDSlotIdx,
      ClimberLiftMtr.Slot2.KIntegralZone,
      ClimberLiftMtr.Slot2.kTimeoutMs);
    
    m_ClimberLiftMtr.configClosedLoopPeakOutput(
      ClimberLiftMtr.Slot2.kPIDSlotIdx, 
      ClimberLiftMtr.Slot2.kClosedLoopPeakOutput);
    
    m_ClimberLiftMtr.configMotionCruiseVelocity(
      ClimberLiftMtr.Slot2.kMotionCruiseVelocity,
      ClimberLiftMtr.Slot2.kTimeoutMs);
    
    m_ClimberLiftMtr.configMotionAcceleration(
      ClimberLiftMtr.Slot2.kMotionAcceleration,
      ClimberLiftMtr.Slot2.kTimeoutMs);
    
    m_ClimberLiftMtr.configMotionSCurveStrength(
      ClimberLiftMtr.Slot2.kSCurveStrength, 
      ClimberLiftMtr.Slot2.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberLiftPos(0 - 515mm)", m_ClimberLiftMtr.getSelectedSensorPosition(0));

    // Update Climber Interaction
    m_ClimberInteraction.setLiftPos(m_ClimberLiftMtr.getSelectedSensorPosition(0));
    m_LiftRotatorOffset = m_ClimberInteraction.getLiftRotateOffset();
    updateLiftSoftLimits();

    SmartDashboard.putNumber("ClimberLift_RotPos", m_ClimberInteraction.getRotatorPos());
    SmartDashboard.putNumber("ClimberLiftRotOffset", m_LiftRotatorOffset);


  }

  public void setClimberLiftSpdCmd(
    double climberLiftSpdCmd,
    TalonFXControlMode ctrlMode){
    
    // Speed Command    
    m_ClimberLiftMtr.set(ctrlMode, climberLiftSpdCmd);
  }

  public void setClimberLiftPosCmd(
    double climberLiftPosCmd,
    TalonFXControlMode ctrlMode){
    
    // mm Command
    m_ClimberLiftMtr.set(
      ctrlMode,
      climberLiftPosCmd);
  }

  //public void setClimberLiftEncoderPos(double climberLiftEncoderPos){
  //  m_ClimberLiftMtr.setSelectedSensorPosition(climberLiftEncoderPos);
  //}

  public boolean isClimberLiftAtPosCmd(){
     boolean posCheck = false;
     boolean velCheck = false;
     boolean liftAtPos = false;
     double posCheckDbl = 0.0;
     double velCheckDbl = 0.0;
     double liftAtPosDbl = 0.0;
     double posTarget = 0.0;
     double posError = 0.0;

    SmartDashboard.putNumber("LiftSlot", m_ClimberLiftActiveSlot);

    posTarget = m_ClimberLiftMtr.getClosedLoopTarget(0);
    posError = m_ClimberLiftMtr.getClosedLoopError(0);
    posCheck = Math.abs(m_ClimberLiftMtr.getClosedLoopError(0)) < ClimberLiftMtr.Slot0.kClosedLoopPosError;
    velCheck = Math.abs(m_ClimberLiftMtr.getSelectedSensorVelocity(0)) < ClimberLiftMtr.Slot0.kClosedLoopPosMaxVel;

    SmartDashboard.putNumber("ClimberLiftPosTarget", posTarget);
    SmartDashboard.putNumber("ClimberLiftPosError", posError);
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
    SmartDashboard.putNumber("ClimberLiftPosCheck", posCheckDbl);
    SmartDashboard.putNumber("ClimberLiftVelCheck", velCheckDbl);
    
    if (posCheck && velCheck){
      liftAtPos = true;
      liftAtPosDbl = 1.0;
    } else {
      liftAtPos = false;
      liftAtPosDbl = 0.0;
    }

    SmartDashboard.putNumber("ClimberLiftAtPos",liftAtPosDbl);
    return liftAtPos;
  }

  public boolean isClimberLiftAtPosCmdCustom(double maxPosError, double maxVelError){
    boolean posCheck = false;
    boolean velCheck = false;
    boolean liftAtPos = false;
    double posCheckDbl = 0.0;
    double velCheckDbl = 0.0;
    double liftAtPosDbl = 0.0;
    double posTarget = 0.0;
    double posError = 0.0;

    SmartDashboard.putNumber("LiftSlot", m_ClimberLiftActiveSlot);

   if (m_ClimberLiftActiveSlot == 2){
     posTarget = m_ClimberLiftMtr.getClosedLoopTarget(2);
     posError = m_ClimberLiftMtr.getClosedLoopError(2);
     posCheck = Math.abs(m_ClimberLiftMtr.getClosedLoopError(2)) < maxPosError;
     velCheck = Math.abs(m_ClimberLiftMtr.getSelectedSensorVelocity(2)) < maxVelError;   
   } else if (m_ClimberLiftActiveSlot == 1){
     posTarget = m_ClimberLiftMtr.getClosedLoopTarget(1);
     posError = m_ClimberLiftMtr.getClosedLoopError(1);
     posCheck = Math.abs(m_ClimberLiftMtr.getClosedLoopError(1)) < maxPosError;
     velCheck = Math.abs(m_ClimberLiftMtr.getSelectedSensorVelocity(1)) < maxVelError;   
   } else {
     posTarget = m_ClimberLiftMtr.getClosedLoopTarget(0);
     posError = m_ClimberLiftMtr.getClosedLoopError(0);
     posCheck = Math.abs(m_ClimberLiftMtr.getClosedLoopError(0)) < maxPosError;
     velCheck = Math.abs(m_ClimberLiftMtr.getSelectedSensorVelocity(0)) < maxVelError;
   }

   SmartDashboard.putNumber("ClimberLiftPosTarget", posTarget);
   SmartDashboard.putNumber("ClimberLiftPosError", posError);
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
   SmartDashboard.putNumber("ClimberLiftPosCheck", posCheckDbl);
   SmartDashboard.putNumber("ClimberLiftVelCheck", velCheckDbl);
   
   if (posCheck && velCheck){
     liftAtPos = true;
     liftAtPosDbl = 1.0;
   } else {
     liftAtPos = false;
     liftAtPosDbl = 0.0;
   }

   SmartDashboard.putNumber("ClimberLiftAtPos",liftAtPosDbl);
   return liftAtPos;
 }

  public void setPIDSlot(int slotID){
    m_ClimberLiftActiveSlot = slotID;
    m_ClimberLiftMtr.selectProfileSlot(slotID, 0);
    m_ClimberLiftMtr.setIntegralAccumulator(0);
  }

  private void updateLiftSoftLimits(){
    double fwdTravelLimit = ClimberLiftMtr.kFwdSoftLimitThd - m_LiftRotatorOffset; 
    double revTravelLimit = ClimberLiftMtr.kRevSoftLimitThd - m_LiftRotatorOffset;
    m_ClimberLiftMtr.configForwardSoftLimitThreshold(fwdTravelLimit);
    m_ClimberLiftMtr.configReverseSoftLimitThreshold(revTravelLimit);
    SmartDashboard.putNumber("ClimberLiftFwdLimit", fwdTravelLimit);
    SmartDashboard.putNumber("ClimberLiftRevLimit", revTravelLimit);
  }
}
