// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import frc.robot.ChargedUp.Arm.Const_Arm;


public class Subsys_Arm extends SubsystemBase {
 
  private final WPI_TalonFX RotateMotor = new WPI_TalonFX(Const_Arm.RotateMotorCanID);
  private final WPI_TalonFX ExtendMotor = new WPI_TalonFX(Const_Arm.ExtendMotorCanID);
  
  /** Creates a new Subsys_Arm. */
    public Subsys_Arm() {
      RotateMotor.set(ControlMode.PercentOutput, 0);
      ExtendMotor.set(ControlMode.PercentOutput, 0);

      RotateMotor.configFactoryDefault();
      ExtendMotor.configFactoryDefault();
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      
    }

    public void MoveArm(double joystick) {
      RotateMotor.set(ControlMode.PercentOutput, joystick, DemandType.ArbitraryFeedForward, 0);
    }
}
