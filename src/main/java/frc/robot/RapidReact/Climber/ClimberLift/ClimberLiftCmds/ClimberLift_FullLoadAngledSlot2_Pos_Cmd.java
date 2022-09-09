// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;

public class ClimberLift_FullLoadAngledSlot2_Pos_Cmd extends CommandBase {
  
  private final ClimberLiftSubSys m_ClimberLiftSubSys;
  private final TalonFXControlMode m_CtrlMode;
  private final double m_ClimberLiftPosCmd;
  private final Timer m_Timer;
  

  public ClimberLift_FullLoadAngledSlot2_Pos_Cmd(
    ClimberLiftSubSys climberLiftSubSys,
    TalonFXControlMode ctrlMode,
    double climberLiftPosCmd) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberLiftSubSys = climberLiftSubSys;
    m_CtrlMode = ctrlMode;
    m_ClimberLiftPosCmd = climberLiftPosCmd;
    m_Timer = new Timer();
    addRequirements(climberLiftSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberLiftSubSys.setPIDSlot(2);
    m_Timer.reset();
    m_Timer.start();

    m_ClimberLiftSubSys.setClimberLiftPosCmd(
      m_ClimberLiftPosCmd,
      m_CtrlMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimberLiftSubSys.setClimberLiftPosCmd(
      m_ClimberLiftPosCmd,
      m_CtrlMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberLiftSubSys.setClimberLiftSpdCmd(
      0.0,
      TalonFXControlMode.PercentOutput); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.hasElapsed(0.5)){
      if (m_ClimberLiftSubSys.isClimberLiftAtPosCmd()){
        return true;
      } else {
        return false;
      }
      /*
      if (m_ClimberLiftSubSys.isClimberLiftAtPosCmdCustom(5, 30){
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
