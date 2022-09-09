// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubSys extends SubsystemBase {
  /** Creates a new Climber_SubSys. 
   * 
   *  This SubSystem primarily keeps track of the climb
   * 
   * 
  */
  private int m_ClimbState = 0;

  public ClimberSubSys() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimbState", m_ClimbState);
  }

  public void setClimbState(int climbState){
    m_ClimbState = climbState;
  }

  public int getClimbState(){
    return m_ClimbState;
  }

  public void resetClimbState(){
    m_ClimbState = 0;
  }

  public void incrementClimbState(){
    m_ClimbState = m_ClimbState+1;
  }
}
