// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.MecanumDrive.SubSys_MecanumDriveTrain;

public class Cmd_SubSys_MecanumDriveTerain extends CommandBase {  
  //Declare Variables 
  private final SubSys_MecanumDriveTrain m_MecanumDriveTrain;
  private DoubleSupplier xControllerStick;
  private DoubleSupplier yControllerStick;
  private DoubleSupplier zControllerStick;
  
  
  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public Cmd_SubSys_MecanumDriveTerain( SubSys_MecanumDriveTrain DriveSys, DoubleSupplier xControllerStick, DoubleSupplier yControllerStick, DoubleSupplier zControllerStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_MecanumDriveTrain = DriveSys;
    addRequirements(m_MecanumDriveTrain);
    this.xControllerStick = xControllerStick;
    this.yControllerStick = yControllerStick;
    this.zControllerStick = zControllerStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_MecanumDriveTrain.MecanumDrive(-xControllerStick.getAsDouble(), -yControllerStick.getAsDouble(), -zControllerStick.getAsDouble());
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