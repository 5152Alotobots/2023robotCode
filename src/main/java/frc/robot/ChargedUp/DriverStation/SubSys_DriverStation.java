// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SubSys_DriverStation extends SubsystemBase {
  /** Creates a new DriverStationSubSys. */

  // Driver Controller
  private XboxController m_DriverController = new XboxController(0);
  
  // Co-Driver Controller
  private XboxController m_CoDriverController = new XboxController(1);
  
  // AuxDriver Controller
  private XboxController m_AuxDriverController = new XboxController(2);

  public JoystickButton IntakeArmLowPosButton = new JoystickButton(m_CoDriverController, 2);
  public JoystickButton IntakeArmHighPosButton = new JoystickButton(m_CoDriverController, 4);
  public JoystickButton IntakeArmDownPosButton = new JoystickButton(m_CoDriverController, 3);

  public JoystickButton LimeLightAutoRotateButton = new JoystickButton(m_AuxDriverController, 2);
  public JoystickButton LimeLightAutoRangeButton = new JoystickButton(m_AuxDriverController, 3);
  
  public JoystickButton LimeLightAutoRotate2HubButton = new JoystickButton(m_DriverController, 10);
  public JoystickButton LimeLightAutoShootButton = new JoystickButton(m_CoDriverController, 10);

  public JoystickButton LiftFwdPosButton = new JoystickButton(m_AuxDriverController, 1);
  //public JoystickButton LiftRevPosButton = new JoystickButton(m_AuxDriverController, 4);

  public JoystickButton ClimbEnableButton = new JoystickButton(m_AuxDriverController, 5);
  public JoystickButton ClimbResetButton = new JoystickButton(m_AuxDriverController, 6);

  public JoystickButton GyroResetButton = new JoystickButton(m_AuxDriverController, 4);

  public JoystickButton CloseHandButton = new JoystickButton(m_CoDriverController, 6);
  public JoystickButton OpenHandButton = new JoystickButton(m_CoDriverController, 5);

  public SubSys_DriverStation() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    
  /**
  ****** Control System Components
  */

  // ---- Drive Subsystem
  // Drive Fwd Axis


  public double HandSensorBtn() {
    boolean buttonValue = m_AuxDriverController.getRawButton(0);
    SmartDashboard.putBoolean("Hand Ready", buttonValue);
    if (buttonValue == true) return 1; else return 0;
  }
}
