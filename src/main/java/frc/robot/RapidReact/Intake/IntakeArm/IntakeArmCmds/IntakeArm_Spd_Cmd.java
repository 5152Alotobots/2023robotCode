// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeArm.IntakeArmCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;

public class IntakeArm_Spd_Cmd extends CommandBase {
  /** Creates a new Intake_Lift_SpdCmd. */

 public final IntakeArmSubSys m_IntakeArmSubSys;
 // public final BooleanSupplier m_IntakeArm_Up;
 // public final BooleanSupplier m_IntakeArm_Down;
  public final double m_IntakeArmSpdCmd;
  public final boolean m_EnableFdbkCtrl;

  public IntakeArm_Spd_Cmd(
    IntakeArmSubSys intakeArmSubSys,
    double intakeArmSpdCmd,
    boolean enableFdbkCtrl){
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeArmSubSys = intakeArmSubSys;
    m_IntakeArmSpdCmd = intakeArmSpdCmd;
    m_EnableFdbkCtrl = enableFdbkCtrl;

    addRequirements(intakeArmSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeArmSubSys.setIntakeArmSpd(m_IntakeArmSpdCmd*0.5, m_EnableFdbkCtrl);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeArmSubSys.setIntakeArmSpd(0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
