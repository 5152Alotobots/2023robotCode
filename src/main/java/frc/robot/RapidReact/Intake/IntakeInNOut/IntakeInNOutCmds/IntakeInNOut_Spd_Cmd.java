// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutSubSys;

public class IntakeInNOut_Spd_Cmd extends CommandBase {
  /** Creates a new Intake_Lift_SpdCmd. */

  public final IntakeInNOutSubSys m_IntakeInNOutSubSys;
  public final double m_IntakeInNOutSpdCmd;
  public final double m_IntakeInNOutLwrSpdCmd;

  public IntakeInNOut_Spd_Cmd(
    IntakeInNOutSubSys intakeInNOutSubSys,
    double intakeInNOutSpdCmd,
    double intakeInNOutLwrSpdCmd){
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeInNOutSubSys = intakeInNOutSubSys;
    m_IntakeInNOutSpdCmd = intakeInNOutSpdCmd;
    m_IntakeInNOutLwrSpdCmd = intakeInNOutLwrSpdCmd;
    addRequirements(intakeInNOutSubSys);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Percent Output
   //m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd, false);
    //m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd, false);
    

    // Speed Command
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd, true);
    m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Percent Output
    //m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd, false);
    //m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd, false);
    

    // Speed Command
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd, true);
    m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd, true);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(0.0,false);
    m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
