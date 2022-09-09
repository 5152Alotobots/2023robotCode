// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeInNOut;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.RapidReact.Intake.IntakeInteraction;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutConstants.IntakeInNOutLwrMtr;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutConstants.IntakeInNOutMtr;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeInNOutSubSys extends SubsystemBase {
  /** Creates a new IntakeInNOutSubSys. */

  private final TalonFX m_IntakeInNOutMtr;
  private final TalonFX m_IntakeInNOutLwrMtr;
  private  boolean m_EnableVelCheck;

  private IntakeInteraction m_IntakeInteraction = new IntakeInteraction();

  public IntakeInNOutSubSys(IntakeInteraction intakeInteraction) {
    m_IntakeInteraction = intakeInteraction;

    m_EnableVelCheck = false;

    // Intake InNOut Motor
    m_IntakeInNOutMtr = new TalonFX(Constants.CAN_IDs.IntakeInNOutMtr_ID);
  
    m_IntakeInNOutMtr.configFactoryDefault();
    m_IntakeInNOutMtr.setInverted(IntakeInNOutMtr.kInverted);
    m_IntakeInNOutMtr.setNeutralMode(IntakeInNOutMtr.kNeutralMode);
    m_IntakeInNOutMtr.configVoltageCompSaturation(IntakeInNOutMtr.kVoltageCompSaturation);
    m_IntakeInNOutMtr.enableVoltageCompensation(IntakeInNOutMtr.kEnableVoltageComp);
    m_IntakeInNOutMtr.configOpenloopRamp(IntakeInNOutMtr.kOpenloopRamp);
    m_IntakeInNOutMtr.configClosedloopRamp(IntakeInNOutMtr.kClosedloopRamp);
            
    // Feedback Sensor
    m_IntakeInNOutMtr.configSelectedFeedbackSensor(
      IntakeInNOutMtr.kFeedbackDevice,
      IntakeInNOutMtr.kPIDSlotIdx, 
			IntakeInNOutMtr.kTimeoutMs);
    
    m_IntakeInNOutMtr.configIntegratedSensorInitializationStrategy(IntakeInNOutMtr.kSensorInitStrategy);
    m_IntakeInNOutMtr.configSelectedFeedbackCoefficient(IntakeInNOutMtr.kFeedbackCoefficient);

    // Velocity Closed Loop
    m_IntakeInNOutMtr.config_kP(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kPID_P);
    
    m_IntakeInNOutMtr.config_kI(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kPID_I);
    
    m_IntakeInNOutMtr.config_kD(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kPID_D);
    
    m_IntakeInNOutMtr.config_kF(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kPID_FF);

    m_IntakeInNOutMtr.config_IntegralZone(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kIZone);
    
    m_IntakeInNOutMtr.configAllowableClosedloopError(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kAllowableError);
    
    m_IntakeInNOutMtr.configMaxIntegralAccumulator(
      IntakeInNOutMtr.kPIDSlotIdx,
      IntakeInNOutMtr.kMaxIAccum);

		// Config the peak and nominal outputs
    m_IntakeInNOutMtr.configNominalOutputForward(IntakeInNOutMtr.kNominalOutputFwd);
	  m_IntakeInNOutMtr.configNominalOutputReverse(IntakeInNOutMtr.kNominalOutputRev);
		m_IntakeInNOutMtr.configPeakOutputForward(IntakeInNOutMtr.kPeakOutputFwd);
    m_IntakeInNOutMtr.configPeakOutputReverse(IntakeInNOutMtr.kPeakOutputRev);
    m_IntakeInNOutMtr.configNeutralDeadband(IntakeInNOutMtr.kNeutralDeadBand);

    // Intake InNOut Lower Motor
    m_IntakeInNOutLwrMtr = new TalonFX(Constants.CAN_IDs.IntakeInNOutLwrMtr_ID);
  
    m_IntakeInNOutLwrMtr.configFactoryDefault();
    m_IntakeInNOutLwrMtr.setInverted(IntakeInNOutLwrMtr.kInverted);
    m_IntakeInNOutLwrMtr.setNeutralMode(IntakeInNOutLwrMtr.kNeutralMode);
    m_IntakeInNOutLwrMtr.configVoltageCompSaturation(IntakeInNOutLwrMtr.kVoltageCompSaturation);
    m_IntakeInNOutLwrMtr.enableVoltageCompensation(IntakeInNOutLwrMtr.kEnableVoltageComp);

    m_IntakeInNOutLwrMtr.configOpenloopRamp(IntakeInNOutLwrMtr.kOpenloopRamp);
    m_IntakeInNOutLwrMtr.configClosedloopRamp(IntakeInNOutLwrMtr.kClosedloopRamp);
                
    // Feedback Sensor
    m_IntakeInNOutLwrMtr.configSelectedFeedbackSensor(
      IntakeInNOutLwrMtr.kFeedbackDevice,
      IntakeInNOutLwrMtr.kPIDSlotIdx, 
      IntakeInNOutLwrMtr.kTimeoutMs);
        
    m_IntakeInNOutLwrMtr.configIntegratedSensorInitializationStrategy(IntakeInNOutLwrMtr.kSensorInitStrategy);
    m_IntakeInNOutLwrMtr.configSelectedFeedbackCoefficient(IntakeInNOutLwrMtr.kFeedbackCoefficient);

    // Velocity Closed Loop
    m_IntakeInNOutLwrMtr.config_kP(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kPID_P);
        
    m_IntakeInNOutLwrMtr.config_kI(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kPID_I);
        
    m_IntakeInNOutLwrMtr.config_kD(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kPID_D);
       
    m_IntakeInNOutLwrMtr.config_kF(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kPID_FF);
    
    m_IntakeInNOutLwrMtr.config_IntegralZone(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kIZone);
    
    m_IntakeInNOutLwrMtr.configAllowableClosedloopError(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kAllowableError);
     
    m_IntakeInNOutLwrMtr.configMaxIntegralAccumulator(
      IntakeInNOutLwrMtr.kPIDSlotIdx,
      IntakeInNOutLwrMtr.kMaxIAccum);

    // Config the peak and nominal outputs
    m_IntakeInNOutLwrMtr.configNominalOutputForward(IntakeInNOutLwrMtr.kNominalOutputFwd);
    m_IntakeInNOutLwrMtr.configNominalOutputReverse(IntakeInNOutLwrMtr.kNominalOutputRev);
    m_IntakeInNOutLwrMtr.configPeakOutputForward(IntakeInNOutLwrMtr.kPeakOutputFwd);
    m_IntakeInNOutLwrMtr.configPeakOutputReverse(IntakeInNOutLwrMtr.kPeakOutputRev);
    m_IntakeInNOutLwrMtr.configNeutralDeadband(IntakeInNOutLwrMtr.kNeutralDeadBand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("InNOutSpd", m_IntakeInNOutMtr.getSelectedSensorVelocity());
    SmartDashboard.putNumber("InNOutLwrSpd", m_IntakeInNOutLwrMtr.getSelectedSensorVelocity());
    
    checkVelAtCmd(m_EnableVelCheck);
    SmartDashboard.putBoolean("InNOutAtSpdCheck", m_IntakeInteraction.getIntakeInNOutAtCmdVel());
  }

  public void setIntakeInNOutSpd(double intakeInNOutSpdCmd, boolean enableFdbkCtrl){
    if(enableFdbkCtrl){
      m_IntakeInNOutMtr.set(TalonFXControlMode.Velocity, intakeInNOutSpdCmd);
    } else {
      m_IntakeInNOutMtr.set(TalonFXControlMode.PercentOutput, intakeInNOutSpdCmd);
    }
  }

  public void setIntakeInNOutLwrSpd(double intakeInNOutLwrSpdCmd, boolean enableFdbkCtrl){
    if(enableFdbkCtrl){
      m_IntakeInNOutLwrMtr.set(TalonFXControlMode.Velocity, intakeInNOutLwrSpdCmd);
    } else {
      m_IntakeInNOutLwrMtr.set(TalonFXControlMode.PercentOutput, intakeInNOutLwrSpdCmd);
    }
  }

  private void checkVelAtCmd(boolean enableVelCheck){
    boolean InNOutAtVel = false;
    boolean InNOutLwrAtVel = false;
    
    if(enableVelCheck){
      // Check IntakeInNOutMtr
      if(m_IntakeInNOutMtr.getControlMode() == 
        ControlMode.Velocity)
      {
        if(m_IntakeInNOutMtr.getClosedLoopError() <=
          IntakeInNOutMtr.kAllowableError){
          InNOutAtVel = true;
        }else{
          InNOutAtVel = false;
        }
      } else {
        InNOutAtVel = false;
      }
    
      // Check IntakeInNOutLwrMtr 
      if(m_IntakeInNOutLwrMtr.getControlMode() == 
        ControlMode.Velocity)
      {
        if(m_IntakeInNOutLwrMtr.getClosedLoopError() <=
          IntakeInNOutLwrMtr.kAllowableError){
          InNOutLwrAtVel = true;
        }else{
          InNOutLwrAtVel = false;
        }
      } else {
        InNOutLwrAtVel = false;
      }
    
      if(InNOutAtVel && InNOutLwrAtVel){
        m_IntakeInteraction.setIntakeInNOutAtCmdVel(true);
      }else{
        m_IntakeInteraction.setIntakeInNOutAtCmdVel(false);
      }
    }else{
      m_IntakeInteraction.setIntakeInNOutAtCmdVel(false);
    }
    SmartDashboard.putBoolean("InNOutAtVel", InNOutAtVel);
    SmartDashboard.putBoolean("InNOutLwrAtVel", InNOutLwrAtVel);
  }

  public void setEnableVelCheck(Boolean enableVelCheck){  
    if(enableVelCheck){
      m_EnableVelCheck = true;
    }else{
      m_EnableVelCheck = false;
    }
  }
}
