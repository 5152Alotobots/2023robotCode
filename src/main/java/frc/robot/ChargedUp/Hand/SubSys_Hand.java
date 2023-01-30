// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Hand;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Hand extends SubsystemBase {
  /** Creates a new SubSys_Hand. */
  public SubSys_Hand() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public final DoubleSolenoid m_handSolenoid = new DoubleSolenoid(
    20, PneumaticsModuleType.CTREPCM, 0, 1);

  public void OpenHand(){
    m_handSolenoid.set(Value.kReverse);
  }

  public void CloseHand(){
    m_handSolenoid.set(Value.kForward);
  }
}
