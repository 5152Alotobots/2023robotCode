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
  private Rev2mDistanceSensor distSensor = new Rev2mDistanceSensor(Port.kMXP);
  double distance;
  public SubSys_DistanceSensor() {
    distSensor.setAutomaticMode(true);
    distSensor.setEnabled(true);
    distSensor.setRangeProfile(RangeProfile.kHighSpeed);
  }
  
  public double GetDistance() {
    SmartDashboard.putNumber("Sent Distance", distance);
    return distance;
    //return distance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = distSensor.getRange();
    SmartDashboard.putNumber("Range", distSensor.getRange());
  }
}
