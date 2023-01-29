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

    // Get the pitch and roll from the Pigeon IMU.
    double pitch = m_PigeonGyro.getRawGyroPitch();
    double roll = m_PigeonGyro.getRawGyroRoll();
    
    // If pitch is greater than 8 or less than -8...
    if (pitch > 8 || pitch < -8) {
        // Drive using the pitch as the speed. (gyro angle * 0.01 if the gyro is 8 degrees off, the robot will drive at 0.08 speed)
        m_DriveTrain.Drive(-pitch * 0.01, 0, 0, false, false, false);
    } else if (roll > 8 || roll < -8) { // If roll is greater than 8 or less than -8...
        // Drive using the roll as the speed.
        m_DriveTrain.Drive(0, -roll * 0.01, 0, false, false, false);
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
