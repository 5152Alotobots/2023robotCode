// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.ChargeStation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;


public class Cmd_AutoBalance extends CommandBase {
  /** Creates a new Cmd_AutoBalance. */
  
  private final SubSys_PigeonGyro m_PigeonGyro;
  private final SubSys_DriveTrain m_DriveTrain; 

  public Cmd_AutoBalance(SubSys_PigeonGyro pigeonGyro, SubSys_DriveTrain driveTrain) {
    m_PigeonGyro = pigeonGyro;
    m_DriveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PigeonGyro, m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * This code will check the angle that the robot is at and drive in the direction that is needed to balance the robot
     */

    if (m_PigeonGyro.isBalanced() != true ){
      //TODO: Switch to the constants defined in SubSys_ChargeStation_Constants after testing initial code
        if (m_PigeonGyro.getRawGyroPitch() > 8){
            m_DriveTrain.Drive(-.05, 0, 0, false, false, false);
        } else if (m_PigeonGyro.getRawGyroPitch() < -8){
            m_DriveTrain.Drive(.05, 0, 0, false, false, false);
        } else if (m_PigeonGyro.getRawGyroRoll() > 8){
            m_DriveTrain.Drive(0, -.05, 0, false, false, false);
        } else if (m_PigeonGyro.getRawGyroRoll() < -8){
            m_DriveTrain.Drive(0, .05, 0, false, false, false);
        }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
