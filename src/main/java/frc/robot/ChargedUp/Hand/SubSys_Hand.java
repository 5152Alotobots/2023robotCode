// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Hand;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Hand extends SubsystemBase {
  /** Creates a new SubSys_Hand. */
  public SubSys_Hand() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public final Solenoid m_handSolenoid = new Solenoid(
    PneumaticsModuleType.CTREPCM, 
    SubSys_Hand_Constants.kHandSolenoidChannel
  );

  public void OpenHand(){
    m_handSolenoid.set(false);
  }

  public void CloseHand(){
    m_handSolenoid.set(true);
  }
}
