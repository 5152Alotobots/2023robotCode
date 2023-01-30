// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.ColorSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class Subsys_ColorSensor extends SubsystemBase {
  /** Creates a new Subsys_ColorSensor. */
  public Subsys_ColorSensor() {}

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      detectedColor = m_colorSensor.getColor();
      
      double IR = m_colorSensor.getIR();

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("IR", IR);
      SmartDashboard.putString("Game Obj ColSens", GetTypeOfGameElement());
  }
  
  public String GetTypeOfGameElement() {
    if (detectedColor.blue < .21) {
      return "Cone";
    }
    if (detectedColor.blue > .23) {
      return "Cube";
    }
    return "N/A";
  }
}
