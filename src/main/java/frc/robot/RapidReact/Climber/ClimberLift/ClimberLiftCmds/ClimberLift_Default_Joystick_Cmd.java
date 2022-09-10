// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;

public class ClimberLift_Default_Joystick_Cmd extends CommandBase {
  /** Creates a new ClimberLiftManual_Cmd. */
  private final ClimberLiftSubSys m_ClimberLiftSubSys;
  private final BooleanSupplier m_PositiveLiftCmd;
  private final BooleanSupplier m_NegativeLiftCmd;

  public ClimberLift_Default_Joystick_Cmd(
    ClimberLiftSubSys climberLiftSubSys,
    BooleanSupplier positiveLiftCmd,
    BooleanSupplier negativeLiveCmd) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberLiftSubSys = climberLiftSubSys;
    m_PositiveLiftCmd = positiveLiftCmd;
    m_NegativeLiftCmd = negativeLiveCmd;
    addRequirements(climberLiftSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberLiftSubSys.setClimberLiftSpdCmd(
      0.0,
      TalonFXControlMode.PercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_PositiveLiftCmd.getAsBoolean()){
      m_ClimberLiftSubSys.setClimberLiftSpdCmd(
        0.4,
        TalonFXControlMode.PercentOutput);
    } else if (m_NegativeLiftCmd.getAsBoolean()){
      m_ClimberLiftSubSys.setClimberLiftSpdCmd(
        -0.4,
        TalonFXControlMode.PercentOutput);
    } else {
      m_ClimberLiftSubSys.setClimberLiftSpdCmd(
        0.0,
        TalonFXControlMode.PercentOutput);
    }
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
    return false;
  }
}
