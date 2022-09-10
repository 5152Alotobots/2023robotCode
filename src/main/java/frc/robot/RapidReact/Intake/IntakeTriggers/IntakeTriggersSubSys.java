// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake.IntakeTriggers;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM_IDs;
import frc.robot.RapidReact.Intake.IntakeInteraction;

public class IntakeTriggersSubSys extends SubsystemBase {
  /** Creates a new IntakeTriggersSubSys. */
  
  private final Servo m_IntakeLeftTriggerServo;
  private final Servo m_IntakeRightTriggerServo;
 
  private IntakeInteraction m_IntakeInteraction = new IntakeInteraction();

  public IntakeTriggersSubSys(IntakeInteraction intakeInteraction) {
    m_IntakeInteraction = intakeInteraction;

  m_IntakeLeftTriggerServo = new Servo(PWM_IDs.IntakeLeftTrigger_ID);
  m_IntakeRightTriggerServo = new Servo(PWM_IDs.IntakeRightTrigger_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeTriggers(boolean intakeLeftTriggerCmd, boolean intakeRightTriggerCmd){
    if (intakeLeftTriggerCmd){
      m_IntakeLeftTriggerServo.set(1.0);
    } else {
      m_IntakeLeftTriggerServo.set(0.6);
    }

    if (intakeRightTriggerCmd){
      m_IntakeRightTriggerServo.set(0.0);
    } else {
      m_IntakeRightTriggerServo.set(0.4);
    }
  }
}
