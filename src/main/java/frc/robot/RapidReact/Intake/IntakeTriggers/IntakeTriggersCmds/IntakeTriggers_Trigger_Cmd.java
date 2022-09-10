// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersSubSys;

public class IntakeTriggers_Trigger_Cmd extends CommandBase {
  /** Creates a new IntakeTriggersActuate_Cmd. */

  private final IntakeTriggersSubSys m_IntakeTriggersSubSys;
  private final Boolean m_IntakeLeftTriggerCmd;
  private final Boolean m_IntakeRightTriggerCmd;

  public IntakeTriggers_Trigger_Cmd(
    IntakeTriggersSubSys intakeTriggersSubSys,
    Boolean intakeLeftTriggerCmd,
    Boolean intakeRightTriggerCmd){

    m_IntakeTriggersSubSys = intakeTriggersSubSys;
    m_IntakeLeftTriggerCmd = intakeLeftTriggerCmd;
    m_IntakeRightTriggerCmd = intakeRightTriggerCmd;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeTriggersSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeTriggersSubSys.setIntakeTriggers(false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeTriggersSubSys.setIntakeTriggers(
      m_IntakeLeftTriggerCmd,
      m_IntakeRightTriggerCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeTriggersSubSys.setIntakeTriggers(false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
