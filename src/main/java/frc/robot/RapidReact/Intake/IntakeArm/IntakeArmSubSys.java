// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeArm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RapidReact.Intake.IntakeInteraction;

public class IntakeArmSubSys extends SubsystemBase {
  /** Creates a new IntakeArmSubSys. */

  private final TalonFX m_IntakeArmMtr;
  private CANCoder m_IntakeArmEncoder;

  private boolean m_EnablePosCheck;  
  private IntakeInteraction m_IntakeInteraction = new IntakeInteraction();
  
  public IntakeArmSubSys(IntakeInteraction intakeInteraction) {
    m_IntakeInteraction = intakeInteraction;
    
    m_EnablePosCheck = false;
    // Initialize IntakeArm Encoder
    m_IntakeArmEncoder = new CANCoder(Constants.CAN_IDs.IntakeArmEnc_ID);
    m_IntakeArmEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_IntakeArmEncoder.configMagnetOffset(-351.211);
    m_IntakeArmEncoder.configSensorDirection(true);

    m_IntakeArmMtr = new TalonFX(Constants.CAN_IDs.IntakeArmMtr_ID);
    m_IntakeArmMtr.setInverted(TalonFXInvertType.Clockwise);
    m_IntakeArmMtr.setNeutralMode(NeutralMode.Brake);
    m_IntakeArmMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    //m_IntakeArmMtr.configRemoteFeedbackFilter(m_IntakeArmEncoder, 0);
    //m_IntakeArmMtr.setSensorPhase(false);
    //m_IntakeArmMtr.setSelectedSensorPosition(1200, 0, 0);
    //m_IntakeArmMtr.setSelectedSensorPosition(1200); //(101.0*4096.0/360.0);
    
    //m_IntakeArmMtr.configIntegratedSensorOffset(0.0);

 		// Config the peak and nominal outputs
    m_IntakeArmMtr.configNominalOutputForward(0);
    m_IntakeArmMtr.configNominalOutputReverse(0);
    m_IntakeArmMtr.configPeakOutputForward(1);
    m_IntakeArmMtr.configPeakOutputReverse(-1);
    m_IntakeArmMtr.configNeutralDeadband(0.01);
        
    m_IntakeArmMtr.configOpenloopRamp(2);
    m_IntakeArmMtr.configClosedloopRamp(2);

    // Config Remote Encoder
    m_IntakeArmMtr.configRemoteFeedbackFilter(m_IntakeArmEncoder, 0);
    m_IntakeArmMtr.setSensorPhase(true);
    //m_IntakeArmMtr.setSelectedSensorPosition(
    //  0,
    //  0,
    //  0);

    m_IntakeArmMtr.configSelectedFeedbackCoefficient(3600.0/4096.0);

    // Position Closed Loop
    m_IntakeArmMtr.configSelectedFeedbackSensor(
      RemoteFeedbackDevice.RemoteSensor0,
      0, 
      0);

    m_IntakeArmMtr.setSelectedSensorPosition(
      m_IntakeArmEncoder.getAbsolutePosition(),
      0,
      0);

    m_IntakeArmMtr.config_kP(
      0,
      1.0);
    
    m_IntakeArmMtr.config_kI(
      0,
      0.13);
    
    m_IntakeArmMtr.config_kD(
      0,
      0);
    
    m_IntakeArmMtr.config_kF(
      0,
      0);

    m_IntakeArmMtr.configMaxIntegralAccumulator(
      0,
      30);

    m_IntakeArmMtr.config_IntegralZone(
      0,
      25);

    m_IntakeArmMtr.configMotionCruiseVelocity(300);
    m_IntakeArmMtr.configMotionAcceleration(150);
    m_IntakeArmMtr.configMotionSCurveStrength(8);

    // Set Position Limits
    m_IntakeArmMtr.configForwardSoftLimitEnable(true);
    m_IntakeArmMtr.configForwardSoftLimitThreshold(1000);
    m_IntakeArmMtr.configReverseSoftLimitEnable(true);
    m_IntakeArmMtr.configReverseSoftLimitThreshold(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmEncoder", m_IntakeArmEncoder.getPosition());
    SmartDashboard.putNumber("ArmInternalEncoder", m_IntakeArmMtr.getSelectedSensorPosition());
    checkPosAtCmd(m_EnablePosCheck);
  }

  public void setIntakeArmSpd(double intakeArmSpdCmd, boolean enableFdbkCtrl){
    if (enableFdbkCtrl){
      m_IntakeArmMtr.set(TalonFXControlMode.Velocity, intakeArmSpdCmd);
    } else {
      m_IntakeArmMtr.set(TalonFXControlMode.PercentOutput, intakeArmSpdCmd);
    }
  }

  public void setIntakeArmPos(double intakeArmPosCmd){
    m_IntakeArmMtr.set(TalonFXControlMode.MotionMagic, intakeArmPosCmd);
  }

  public boolean isIntakeAtPosCmd(){
    boolean posCheck = m_IntakeArmMtr.getClosedLoopError(0)< 1;
    boolean velCheck = m_IntakeArmMtr.getSelectedSensorVelocity(0) < 1;

    if (posCheck && velCheck){
      return true;
    } else {
      return false;
    }
  }

  public void setEnablePosCheck(Boolean enablePosCheck){  
    if(enablePosCheck){
      m_EnablePosCheck = true;
    }else{
      m_EnablePosCheck = false;
    }
  }

  public void checkPosAtCmd(boolean enablePosCheck){
    if(enablePosCheck){
      if(isIntakeAtPosCmd()){
        m_IntakeInteraction.setIntakeArmAtCmdPos(true);
      }else{
        m_IntakeInteraction.setIntakeArmAtCmdPos(false);
      }
    }else{
      m_IntakeInteraction.setIntakeArmAtCmdPos(false);
    }
  }
}
