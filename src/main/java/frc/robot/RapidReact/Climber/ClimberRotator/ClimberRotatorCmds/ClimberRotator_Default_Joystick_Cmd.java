// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorCmds;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;

public class ClimberRotator_Default_Joystick_Cmd extends CommandBase {
  /** Creates a new ClimberRotatorManual_Cmd. */
  private final ClimberRotatorSubSys m_ClimberRotatorSubSys;
  private final BooleanSupplier m_RotatorPositiveCmd;
  private final BooleanSupplier m_RotatorNegativeCmd;

  public ClimberRotator_Default_Joystick_Cmd(
    ClimberRotatorSubSys climberRotatorSubSys,
    BooleanSupplier rotatorPositiveCmd,
    BooleanSupplier rotatorNegativeCmd
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberRotatorSubSys = climberRotatorSubSys;
    m_RotatorPositiveCmd = rotatorPositiveCmd;
    m_RotatorNegativeCmd = rotatorNegativeCmd;
    addRequirements(climberRotatorSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(0.0, TalonFXControlMode.PercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_RotatorPositiveCmd.getAsBoolean()){
      m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(1.0, TalonFXControlMode.PercentOutput);
    } else if (m_RotatorNegativeCmd.getAsBoolean()){
      m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(-.5, TalonFXControlMode.PercentOutput);
    } else {
      m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(0.0, TalonFXControlMode.PercentOutput);
    }
    
    /*
    if (m_RotatorPositiveCmd.getAsBoolean()){
      m_ClimberRotatorSubSys.setRotatorPosition(10000);
    } else if (m_RotatorNegativeCmd.getAsBoolean()){
      m_ClimberRotatorSubSys.setRotatorPosition(-10000);
    } else {
      m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(0, TalonFXControlMode.PercentOutput);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberRotatorSubSys.setClimberRotatorSpdCmd(0.0, TalonFXControlMode.PercentOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
