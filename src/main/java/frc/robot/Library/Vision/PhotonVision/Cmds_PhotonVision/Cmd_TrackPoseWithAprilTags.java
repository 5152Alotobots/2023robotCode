// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.Vision.PhotonVision.Cmds_PhotonVision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.MecanumDrive.*;
import frc.robot.Library.Vision.PhotonVision.*;

public class Cmd_TrackPoseWithAprilTags extends CommandBase {  
  //Declare Variables 
  private final SubSys_PhotonVision m_PhotonVisionSubsytem;
  private final SubSys_MecanumDriveTrain m_DriveSubsystem;
  // Change this to match the name of your camera

  PhotonCamera cameraFront = new PhotonCamera("OV5647");
  //PhotonCamera cameraRear = new PhotonCamera("OV5647");
  
  /**Constructor 
   * @param DriveSys The mecanum drive train subsystem
   * @param PhotonVisionSys The photon vision subsystem
   */
  public Cmd_TrackPoseWithAprilTags(SubSys_PhotonVision PhotonVisionSys, SubSys_MecanumDriveTrain MecanumDriveSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PhotonVisionSubsytem = PhotonVisionSys;
    m_DriveSubsystem = MecanumDriveSys;
    addRequirements(m_PhotonVisionSubsytem, m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Use values to calculate pose
    m_PhotonVisionSubsytem.getRobotPoseFromAprilTagVision(cameraFront, null); //TODO: Pass prev pose into command
    //TODO: Use this https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/Drivetrain.java
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