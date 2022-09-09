// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeArm.IntakeArmCmds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;

public class IntakeArm_PosHold_Cmd extends CommandBase {
  /** Creates a new Intake_Lift_SpdCmd. */

  public final IntakeArmSubSys m_IntakeArmSubSys;
  // public final BooleanSupplier m_IntakeArm_Up;
  // public final BooleanSupplier m_IntakeArm_Down;
  public final double m_IntakeArmPosCmd;
  public final DoubleSupplier m_IntakeArmAxis;

  public IntakeArm_PosHold_Cmd(
    IntakeArmSubSys intakeArmSubSys,
    double intakeArmPosCmd,
    DoubleSupplier intakeArmAxis){

    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeArmSubSys = intakeArmSubSys;
    m_IntakeArmPosCmd = intakeArmPosCmd;
    m_IntakeArmAxis = intakeArmAxis;

    addRequirements(intakeArmSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeArmSubSys.setIntakeArmPos(m_IntakeArmPosCmd);
    m_IntakeArmSubSys.setEnablePosCheck(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeArmSubSys.setIntakeArmPos(m_IntakeArmPosCmd);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeArmSubSys.setEnablePosCheck(false);
    m_IntakeArmSubSys.setIntakeArmSpd(0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_IntakeArmAxis.getAsDouble())> 0.1){
      return true;
    } else {
      return false;
    }
  }
}
