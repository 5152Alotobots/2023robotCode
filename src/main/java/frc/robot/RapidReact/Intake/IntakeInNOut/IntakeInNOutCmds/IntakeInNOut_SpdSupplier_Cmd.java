// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutCmds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutSubSys;

public class IntakeInNOut_SpdSupplier_Cmd extends CommandBase {
  /** Creates a new Intake_Lift_SpdCmd. */

  public final IntakeInNOutSubSys m_IntakeInNOutSubSys;
  public final DoubleSupplier m_IntakeInNOutSpdCmd;
  public final DoubleSupplier m_IntakeInNOutLwrSpdCmd;

  public IntakeInNOut_SpdSupplier_Cmd(
    IntakeInNOutSubSys intakeInNOutSubSys,
    DoubleSupplier intakeInNOutSpdCmd,
    DoubleSupplier intakeInNOutLwrSpdCmd){
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeInNOutSubSys = intakeInNOutSubSys;
    m_IntakeInNOutSpdCmd = intakeInNOutSpdCmd;
    m_IntakeInNOutLwrSpdCmd = intakeInNOutLwrSpdCmd;
    addRequirements(intakeInNOutSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Speed Command
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd.getAsDouble(), true);
    m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd.getAsDouble(), true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Speed Command
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(m_IntakeInNOutSpdCmd.getAsDouble(), true);
    m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(m_IntakeInNOutLwrSpdCmd.getAsDouble(), true);
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
