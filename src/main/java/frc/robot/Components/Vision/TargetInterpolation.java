// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.RobotSettings;

/** Add your docs here. */
public class TargetInterpolation {
  
  private int m_Index = 0;
  private double m_Frac = 0;

  // Determine Index and Fraction
  public void calcInterpIndexAndFrac(double distance){
    /*int i=0;
    for(i=0;i<RobotSettings.Shooting.kShootDistance.length;i++){
      if(distance<=RobotSettings.Shooting.kShootDistance[i]){
        m_Index = i;
        break;
      }else{
        m_Index = i+1;
      }
    }
    if(m_Index == 0){
      m_Frac = 0;
    } else if(m_Index >= RobotSettings.Shooting.kShootDistance.length){
      m_Frac = 0;
    } else {
      m_Frac = (RobotSettings.Shooting.kShootDistance[m_Index]-distance)/
      (RobotSettings.Shooting.kShootDistance[m_Index+1]-RobotSettings.Shooting.kShootDistance[m_Index]);
    }
    */
    if(distance<RobotSettings.Shooting.kShootDistance[0]){
      m_Index = 0;
      m_Frac = 0.0;
    }else if(distance<RobotSettings.Shooting.kShootDistance[1]){
      m_Index = 0;
      m_Frac = (distance-RobotSettings.Shooting.kShootDistance[m_Index])/
      (RobotSettings.Shooting.kShootDistance[m_Index+1]-RobotSettings.Shooting.kShootDistance[m_Index]);
    }else if(distance<RobotSettings.Shooting.kShootDistance[2]){
      m_Index = 1;
      m_Frac = (distance-RobotSettings.Shooting.kShootDistance[m_Index])/
      (RobotSettings.Shooting.kShootDistance[m_Index+1]-RobotSettings.Shooting.kShootDistance[m_Index]);
    }else if(distance<RobotSettings.Shooting.kShootDistance[3]){
      m_Index = 2;
      m_Frac = (distance-RobotSettings.Shooting.kShootDistance[m_Index])/
      (RobotSettings.Shooting.kShootDistance[m_Index+1]-RobotSettings.Shooting.kShootDistance[m_Index]);
    }else{
      m_Index = 3;
      m_Frac = 0.0;
    }  
    
    SmartDashboard.putNumber("Index", m_Index);
    SmartDashboard.putNumber("Frac", m_Frac);
  }

  // Determine ArmAngle
  public double calcArmAngle(){
    double armAngle = 0.0;
    if(m_Index >= RobotSettings.Shooting.kShootDistance.length-1){
      armAngle = RobotSettings.Shooting.kShootingAngle[RobotSettings.Shooting.kShootDistance.length-1];
    } else {
      armAngle = RobotSettings.Shooting.kShootingAngle[m_Index]+
      m_Frac*(RobotSettings.Shooting.kShootingAngle[m_Index+1]-RobotSettings.Shooting.kShootingAngle[m_Index]);  
    }
    return armAngle;
  } 

  // Interp and Calc Arm angle
  public double interpCalcArmAngle(DoubleSupplier distance){
    calcInterpIndexAndFrac(distance.getAsDouble());
    return calcArmAngle();
  }

  // Determine InNOutVel
  public double calcInNOutVel(){
    double inNOutVel = 0.0;
    if(m_Index >= RobotSettings.Shooting.kShootDistance.length-1){
      inNOutVel = RobotSettings.Shooting.kShootingInNOutVel[RobotSettings.Shooting.kShootDistance.length-1];
    } else{
      inNOutVel = RobotSettings.Shooting.kShootingInNOutVel[m_Index]+
      m_Frac*(RobotSettings.Shooting.kShootingInNOutVel[m_Index+1]-RobotSettings.Shooting.kShootingInNOutVel[m_Index]);  
    }
      return inNOutVel;
  }
  
  // Interp and Calc InNOutVel
  public double interpCalcInNOutVel(DoubleSupplier distance){
    calcInterpIndexAndFrac(distance.getAsDouble());
    return calcInNOutVel();
  }

  // Determine InNOutLwrVel
  public double calcInNOutLwrVel(){
    double inNOutLwrVel = 0.0;
    if(m_Index >= RobotSettings.Shooting.kShootDistance.length-1){
      inNOutLwrVel = RobotSettings.Shooting.kShootingInNOutLwrVel[RobotSettings.Shooting.kShootDistance.length-1];
    } else{
      inNOutLwrVel = RobotSettings.Shooting.kShootingInNOutLwrVel[m_Index]+
      m_Frac*(RobotSettings.Shooting.kShootingInNOutLwrVel[m_Index+1]-RobotSettings.Shooting.kShootingInNOutLwrVel[m_Index]);  
    }
      return inNOutLwrVel;
  }

  // Interp and Calc InNOutLwrVel
  public double interpCalcInNOutLwrVel(DoubleSupplier distance){
    calcInterpIndexAndFrac(distance.getAsDouble());
    return calcInNOutLwrVel();
  }
}
