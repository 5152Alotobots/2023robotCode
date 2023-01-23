// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.Vision.PhotonVision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SubSys_PhotonVisionConstants {

  public static class FieldConstants {
    public static final double length = 13.0;
    public static final double width = 8.0;
  }
    
    public final static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4);
    public final static double TARGET_HEIGHT_METERS = Units.inchesToMeters(9.5);
    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  public static class AprilTagPoseEstimatorConstants {
    public static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                    0, 0,
                    0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.


      public final static AprilTag tag18 =
              new AprilTag(
                      18,
                      new Pose3d(
                              new Pose2d(
                                      FieldConstants.length,
                                      FieldConstants.width / 2.0,
                                      Rotation2d.fromDegrees(180))));
      public final static AprilTag tag01 =
              new AprilTag(
                      01,
                      new Pose3d(
                              new Pose2d(
                                0.0, FieldConstants.width / 2.0, 
                                Rotation2d.fromDegrees(0.0))));

    }
  }
