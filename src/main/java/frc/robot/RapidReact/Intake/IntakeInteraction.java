// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Intake;

/** Add your docs here. */
public class IntakeInteraction {
    private boolean m_IntakeArmAtCmdPos;
    private boolean m_IntakeInNOutAtCmdVel;

    public void setIntakeArmAtCmdPos(boolean atPos){
        m_IntakeArmAtCmdPos = atPos;
    }

    public void setIntakeInNOutAtCmdVel(boolean atSpd){
        m_IntakeInNOutAtCmdVel = atSpd;
    }

    public boolean getIntakeArmAtCmdPos(){
        return m_IntakeArmAtCmdPos;
    }

    public boolean getIntakeInNOutAtCmdVel(){
        return m_IntakeInNOutAtCmdVel;
    }
}
