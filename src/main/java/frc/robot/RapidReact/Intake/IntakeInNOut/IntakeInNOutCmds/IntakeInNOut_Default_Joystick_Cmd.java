// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutCmds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotSettings;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutSubSys;

public class IntakeInNOut_Default_Joystick_Cmd extends CommandBase {
  /** Creates a new Intake_InNOut_SpdCmd. */

  public final IntakeInNOutSubSys m_IntakeInNOutSubSys;
  public final BooleanSupplier m_IntakeInNOut_Intake;
  public final DoubleSupplier m_IntakeInNOut_ShortShotAxis;
  public final DoubleSupplier m_IntakeInNOut_LongShotAxis;
  public final boolean m_IntakeInNOut_EnableFdbkCtrl;

  public IntakeInNOut_Default_Joystick_Cmd(
    IntakeInNOutSubSys intakeInNOutSubSys, 
    BooleanSupplier intakeInNOut_Intake,
    DoubleSupplier intakeInNOut_ShortShotAxis,
    DoubleSupplier intakeInNOut_LongShotAxis,
    boolean intakeInNOut_EnableFdbkCtrl) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeInNOutSubSys = intakeInNOutSubSys;
    m_IntakeInNOut_Intake = intakeInNOut_Intake;
    m_IntakeInNOut_ShortShotAxis = intakeInNOut_ShortShotAxis;
    m_IntakeInNOut_LongShotAxis = intakeInNOut_LongShotAxis;
    m_IntakeInNOut_EnableFdbkCtrl = intakeInNOut_EnableFdbkCtrl;
    addRequirements(intakeInNOutSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    // Percent Output
    if (m_IntakeInNOut_Intake.getAsBoolean()){
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(0.4, m_IntakeInNOut_EnableFdbkCtrl);
    } else if (m_IntakeInNOut_ShortShot.getAsBoolean()){
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(-0.36, m_IntakeInNOut_EnableFdbkCtrl);
    } else if (m_IntakeInNOut_LongShot.getAsBoolean()){
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(-1, m_IntakeInNOut_EnableFdbkCtrl);
    } else {
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(0.0, false);
    }
    */

    // Velocity Command
    if (m_IntakeInNOut_Intake.getAsBoolean()){
      m_IntakeInNOutSubSys.setEnableVelCheck(false);
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(RobotSettings.Intake.kIntakeVel, true);
      m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(RobotSettings.Intake.kIntakeLwrVel, true);
    } else if (m_IntakeInNOut_ShortShotAxis.getAsDouble()> 0.1){
      m_IntakeInNOutSubSys.setEnableVelCheck(true);
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(
        m_IntakeInNOut_ShortShotAxis.getAsDouble()*RobotSettings.Shooting.kShootLowGoalVel,
        true);
      m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(
        m_IntakeInNOut_ShortShotAxis.getAsDouble()*RobotSettings.Shooting.kShootLwrLowGoalVel,
        true);
    } else if (m_IntakeInNOut_LongShotAxis.getAsDouble()> 0.1){
      m_IntakeInNOutSubSys.setEnableVelCheck(true);
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(
        m_IntakeInNOut_LongShotAxis.getAsDouble()*RobotSettings.Shooting.kShootHighGoalVel,
        true);
      m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(
        m_IntakeInNOut_LongShotAxis.getAsDouble()*RobotSettings.Shooting.kShootLwrHighGoalVel,
        true);
    } else {
      m_IntakeInNOutSubSys.setEnableVelCheck(false);
      m_IntakeInNOutSubSys.setIntakeInNOutSpd(0.0, false);
      m_IntakeInNOutSubSys.setIntakeInNOutLwrSpd(0.0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeInNOutSubSys.setEnableVelCheck(false);
    m_IntakeInNOutSubSys.setIntakeInNOutSpd(0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
