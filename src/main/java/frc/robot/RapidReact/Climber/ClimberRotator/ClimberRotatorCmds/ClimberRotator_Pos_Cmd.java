// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorCmds;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;

public class ClimberRotator_Pos_Cmd extends CommandBase {
  
  private final ClimberRotatorSubSys m_ClimberRotatorSubSys;
  private final TalonFXControlMode m_CtrlMode;
  private final double m_ClimberLiftPosCmd;
  private final boolean m_FFEnable;
  private final double m_AngleOffset;
  private final Timer m_Timer;

  public ClimberRotator_Pos_Cmd(
    ClimberRotatorSubSys climberRotatorSubSys,
    TalonFXControlMode ctrlMode,
    double climberLiftPosCmd,
    boolean ffEnable,
    double angOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberRotatorSubSys = climberRotatorSubSys;
    m_CtrlMode = ctrlMode;
    m_ClimberLiftPosCmd = climberLiftPosCmd;
    m_FFEnable = ffEnable;
    m_AngleOffset = angOffset;
    m_Timer = new Timer();
    addRequirements(climberRotatorSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_ClimberRotatorSubSys.setClimberRotatorPosCmd(
      m_ClimberLiftPosCmd,
      m_CtrlMode,
      m_FFEnable,
      m_AngleOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimberRotatorSubSys.setClimberRotatorPosCmd(
      m_ClimberLiftPosCmd,
      m_CtrlMode,
      m_FFEnable,
      m_AngleOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(0.0, TalonFXControlMode.PercentOutput); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Timer.hasElapsed(0.5)){
      if (m_ClimberRotatorSubSys.isClimberRotatorAtPosCmd()){
        return true;
      }else{
        return false;
      }
      /*
      if (m_ClimberRotatorSubSys.isClimberRotatorAtPosCmdCustom(5, 5){
        return true;
      } else {
        return false;
      }
      */
    }else{
      return false;
    }
  }
}
