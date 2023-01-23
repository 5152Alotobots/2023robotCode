// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.Vision.PhotonVision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.DriveTrains.MecanumDrive.SubSys_MecanumDriveTrain;
import frc.robot.Library.Vision.PhotonVision.SubSys_PhotonVisionConstants;
import frc.robot.Library.Vision.PhotonVision.SubSys_PhotonVisionConstants.FieldConstants;
import frc.robot.Library.Vision.PhotonVision.SubSys_PhotonVisionConstants.AprilTagPoseEstimatorConstants;

public class SubSys_PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVisionSubsytem. */
  public SubSys_PhotonVision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /** VISION */
    public double getRangeToTag(PhotonPipelineResult result){
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
          SubSys_PhotonVisionConstants.CAMERA_HEIGHT_METERS,
          SubSys_PhotonVisionConstants.TARGET_HEIGHT_METERS,
          SubSys_PhotonVisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
      return range;
    }

    /** Calculate strafe speed */
    public double getVisionStrafeSpeed(SubSys_MecanumDriveTrain m_MecanumDriveTrain, PhotonPipelineResult result){
      double strafeSpeed;
      if (result.hasTargets()) {
        strafeSpeed = -m_MecanumDriveTrain.controller.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        strafeSpeed = 0;
      }
      return strafeSpeed;
    }
    
    /** Calculate forward speed */
    public double getVisionForwardSpeed(SubSys_MecanumDriveTrain m_MecanumDriveTrain, PhotonPipelineResult result){
      double forwardSpeed;
      if (result.hasTargets()) {
        forwardSpeed = -m_MecanumDriveTrain.controller.calculate(getRangeToTag(result), SubSys_PhotonVisionConstants.GOAL_RANGE_METERS);
      } else {
        forwardSpeed = 0;
      }
      return forwardSpeed;
    }

  
    //TODO: implement pose estimator found here: https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/PhotonCameraWrapper.java
    //TODO: TEST CODE
    public Optional<EstimatedRobotPose> getRobotPoseFromAprilTagVision(PhotonCamera cam, Pose2d prevEstimatedRobotPose){

      // Load field layout from resource file. Handle IOException by reporting it to driver station before robot crashes.
      AprilTagFieldLayout atfl;
      try {
        atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException exception) {
        DriverStation.reportError("Failed to load AprilTagFieldLayout from JSON", false);
        throw new RuntimeException(exception);
    }
      boolean isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
      atfl.setOrigin(isRedAlliance ? OriginPosition.kRedAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);

      // Create pose estimator
      var photonPoseEstimator =
              new PhotonPoseEstimator(
                      atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, AprilTagPoseEstimatorConstants.robotToCam);

      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();                
                    
    }

    
}

