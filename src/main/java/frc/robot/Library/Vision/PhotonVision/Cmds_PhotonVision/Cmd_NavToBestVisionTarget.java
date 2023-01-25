// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.Vision.PhotonVision.Cmds_PhotonVision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.MecanumDrive.*;
import frc.robot.Library.Vision.PhotonVision.*;

public class Cmd_NavToBestVisionTarget extends CommandBase {  
  //Declare Variables 
  private final SubSys_MecanumDriveTrain m_DriveSubsystem;
  private final SubSys_PhotonVision m_PhotonVisionSubsytem;
  // Change this to match the name of your camera

  PhotonCamera cameraFront = new PhotonCamera("OV5647");
  //PhotonCamera cameraRear = new PhotonCamera("OV5647");
  
  /**Constructor 
   * @param DriveSys The mecanum drive train subsystem
   * @param PhotonVisionSys The photon vision subsystem
   */
  public Cmd_NavToBestVisionTarget( SubSys_MecanumDriveTrain DriveSys, SubSys_PhotonVision PhotonVisionSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = DriveSys;
    m_PhotonVisionSubsytem = PhotonVisionSys;
    addRequirements(m_DriveSubsystem, m_PhotonVisionSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    /* FRONT CAMERA */
    var resultFront = cameraFront.getLatestResult();
    
    //Use values to drive robot
    m_DriveSubsystem.MecanumDrive(m_PhotonVisionSubsytem.getVisionStrafeSpeed(m_DriveSubsystem, resultFront)*0.5, m_PhotonVisionSubsytem.getVisionForwardSpeed(m_DriveSubsystem, resultFront)*0.4, 0);
    //SmartDashboard.putNumber("Rotation", rotationSpeed);
    //SmartDashboard.putNumber("Controller Z POS", zControllerStick.getAsDouble());

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