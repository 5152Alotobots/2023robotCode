// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.DriverStation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Components.DriverStation.JoystickUtilities;
import frc.robot.RapidReact.Intake.IntakeInteraction;

public class DriverStationSubSys extends SubsystemBase {
  /** Creates a new DriverStationSubSys. */

  // Driver Controller
  private XboxController m_DriverController = new XboxController(0);
  
  // Co-Driver Controller
  private XboxController m_CoDriverController = new XboxController(1);
  
  // AuxDriver Controller
  private XboxController m_AuxDriverController = new XboxController(2);

  // IntakeInteraction
  private IntakeInteraction m_IntakeInteraction = new IntakeInteraction();

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

  public DriverStationSubSys(IntakeInteraction intakeInteraction) {
    m_IntakeInteraction = intakeInteraction;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Check if InNOut is at Velocity
    if(m_IntakeInteraction.getIntakeInNOutAtCmdVel()){
      m_CoDriverController.setRumble(RumbleType.kRightRumble, 0);
    }else{
      m_CoDriverController.setRumble(RumbleType.kRightRumble, 0);
    }
    SmartDashboard.putBoolean("DS_IntakeInNOutAtCmdVel", m_IntakeInteraction.getIntakeInNOutAtCmdVel());

    // Check if IntakeArm is at Position
    if(m_IntakeInteraction.getIntakeArmAtCmdPos()){
      m_CoDriverController.setRumble(RumbleType.kLeftRumble, 0);  
    }else{
      m_CoDriverController.setRumble(RumbleType.kLeftRumble, 0);
    }
    
    
    SmartDashboard.putBoolean("DS_IntakeArmAtCmdPos", m_IntakeInteraction.getIntakeArmAtCmdPos());
  }
    
  /**
  ****** Control System Components
  */

  // ---- Drive Subsystem
  // Drive Fwd Axis
  public double DriveFwdAxis(){
    return -m_DriverController.getRawAxis(1);
  }

  

  // Drive Strafe Axis
  public double DriveStrAxis(){
    return -m_DriverController.getRawAxis(0);
  }

  // Drive Rotate Axis
  public double DriveRotAxis(){
    return -m_DriverController.getRawAxis(4);
  }

  // Drive RotateLeftPoint
  public boolean RotateLeftPt(){
    return m_DriverController.getRawButton(5);
  }

  // Drive RotateRightPoint
  public boolean RotateRightPt(){
    return m_DriverController.getRawButton(6);
  }
  
  /*
  ***** Rapid React Components
  */

  // ---- Intake
  // ------ IntakeInNOut

  // IntakeInNOut_Intake
  public boolean IntakeInNOut_Intake(){
    return m_CoDriverController.getRawButton(1);
  }
  
  /*
  // IntakeInOut_ShortShot
  public boolean IntakeInNOut_ShortShot(){
    return m_CoDriverController.getRawButton(2);
  }
  */

  // IntakeInOut_ShortShot Axis
  public double IntakeInNOut_ShortShotAxis(){
    return m_CoDriverController.getRightTriggerAxis();
  }

  /*
  // IntakeInOut_LongShot
    public boolean IntakeInNOut_LongShot(){
    return m_CoDriverController.getRawButton(4);
  }
  */

  // IntakeInNOut_LongShotAxis
  public double IntakeInNOut_LongShotAxis(){
    return m_CoDriverController.getLeftTriggerAxis();
  }

  // ------IntakeArm

  //IntakeArm_Axis
  public double IntakeArm_Axis(){
    return m_CoDriverController.getRawAxis(1);
  }

  // IntakeLeftTrigger
  public boolean IntakeLeftTrigger(){
    return m_CoDriverController.getRawButton(5);
  }

  // IntakeLeftTriggerAxis
  public double IntakeLeftTriggerAxis(){
    return m_DriverController.getRawAxis(2);
  }

  // IntakeRightTrigger
  public boolean IntakeRightTrigger(){
    return m_CoDriverController.getRawButton(6);
  }

  // IntakeRightTriggerAxis
  public double IntakeRightTriggerAxis(){
    return m_DriverController.getRawAxis(3);
  }

  // ---- Climber
  // ------ Climber Lift
  
  // Climber Lift Positive
  public boolean ClimberLiftPositiveBtn(){
    return m_DriverController.getRawButton(4);
  }

  // Climber Lift Negative
  public boolean ClimberLiftNegativeBtn(){
    return m_DriverController.getRawButton(1);
  }
  
  // ------ Climber Rotator
  public boolean ClimberLiftRotatorPositiveBtn(){
    return m_DriverController.getRawButton(2);
  }

  public boolean ClimberLiftRotatorNegativeBtn(){
    return m_DriverController.getRawButton(3);
  }
}
