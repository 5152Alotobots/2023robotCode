// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.DistanceSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;


public class SubSys_DistanceSensor extends SubsystemBase {
  /** Creates a new SubSys_DistanceSensor. */
  // private Rev2mDistanceSensor distSens = new Rev2mDistanceSensor();

  public SubSys_DistanceSensor() {
    // distSens.setAutomaticMode(true);
    // distSens.setEnabled(true);
    // distSens.setRangeProfile(RangeProfile.kHighSpeed);
  }
  
  public double GetDistance() {
    return 1.25;
    //return distance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // public double distance = distSens.getRange();
    // SmartDashboard.putNumber("Range", distSens.getRange());


  }
}
