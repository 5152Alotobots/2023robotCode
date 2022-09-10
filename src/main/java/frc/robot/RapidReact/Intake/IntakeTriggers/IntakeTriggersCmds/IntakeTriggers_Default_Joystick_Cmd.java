// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersCmds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersSubSys;

public class IntakeTriggers_Default_Joystick_Cmd extends CommandBase {
  /** Creates a new IntakeTriggersActuate_Cmd. */

  private final IntakeTriggersSubSys m_IntakeTriggersSubSys;
  private final DoubleSupplier m_IntakeLeftTriggerAxis;
  private final DoubleSupplier m_IntakeRightTriggerAxis;

  public IntakeTriggers_Default_Joystick_Cmd(
    IntakeTriggersSubSys intakeTriggersSubSys,
    DoubleSupplier intakeLeftTriggerAxis,
    DoubleSupplier intakeRightTriggerAxis){

    m_IntakeTriggersSubSys = intakeTriggersSubSys;
    m_IntakeLeftTriggerAxis = intakeLeftTriggerAxis;
    m_IntakeRightTriggerAxis = intakeRightTriggerAxis;
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
      (m_IntakeLeftTriggerAxis.getAsDouble() > 0.25),
      (m_IntakeRightTriggerAxis.getAsDouble() > 0.25));
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
