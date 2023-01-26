// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/** 
 * * For better comments extention (Add above package)
 * * importandt 
 * ! Deprecitated 
 * ? question 
 * TODO:
 * @param myParam[Sqr_brackets_for_multipl_Sim_elements(likemotors)] (Used to define params)
 */
package frc.robot.ChargedUp.MecanumDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.ChargedUp.MecanumDrive.Const_MecanumDrive;


import org.ejml.equation.Variable;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsys_MecanumDrive extends SubsystemBase {
/**
 * *Declare Motors 
 * @param k[Motor]Port configure to fit needs 
 * (You may need to use the phoenix tuner)
 */
  private final WPI_VictorSPX m_frontLeftMotor = new WPI_VictorSPX(Const_MecanumDrive.k_FrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeftMotor = new WPI_VictorSPX(Const_MecanumDrive.k_RearLeftMotorPort);
  private final WPI_VictorSPX m_frontRightMotor = new WPI_VictorSPX(Const_MecanumDrive.k_FrontRightMotorPort);
  private final WPI_VictorSPX m_rearRightMotor = new WPI_VictorSPX(Const_MecanumDrive.k_RearRightMotorPort);
  
 

//*Declare Encoders  
  /** 
   *  The front-left-side drive encoder @param k[Encoder]Reversed boolean if that encoder reversed or not 
   * True = isReversed Flase = notReversed
   *
  */
  /** Sets the front left drive MotorController to a voltage. */
  public Subsys_MecanumDrive() {
    m_frontRightMotor.setInverted(true);
    m_frontLeftMotor.setInverted(true);
    m_rearLeftMotor.setInverted(true);
    m_rearRightMotor.setInverted(false);
  }
  //initialize Mecanum Drive 
  //TODO see if this is correct
  private final MecanumDrive m_MecanumDrive = new MecanumDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);
  
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeftMotor.setVoltage(volts.frontLeftVoltage);
    m_rearLeftMotor.setVoltage(volts.rearLeftVoltage);
    m_frontRightMotor.setVoltage(volts.frontRightVoltage);
    m_rearRightMotor.setVoltage(volts.rearRightVoltage);
  }

  // private final Encoder m_frontLeftEncoder =
  //     new Encoder(
  //         DriveConstants.k_FrontLeftEncoderPorts[0],
  //         DriveConstants.k_FrontLeftEncoderPorts[1],
  //         DriveConstants.k_FrontLeftEncoderReversed);

  // // The rear-left-side drive encoder
  // private final Encoder m_rearLeftEncoder =
  //     new Encoder(
  //         DriveConstants.k_RearLeftEncoderPorts[0],
  //         DriveConstants.k_RearLeftEncoderPorts[1],
  //         DriveConstants.k_RearLeftEncoderReversed);

  // // The front-right--side drive encoder
  // private final Encoder m_frontRightEncoder =
  //     new Encoder(
  //         DriveConstants.k_FrontRightEncoderPorts[0],
  //         DriveConstants.k_FrontRightEncoderPorts[1],
  //         DriveConstants.k_FrontRightEncoderReversed);

  // // The rear-right-side drive encoder
  // private final Encoder m_rearRightEncoder =
  //     new Encoder(
  //         DriveConstants.k_RearRightEncoderPorts[0],
  //         DriveConstants.k_RearRightEncoderPorts[1],
  //         DriveConstants.k_RearRightEncoderReversed);
//*Declare Gyro
  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

// Locations of the wheels relative to the robot center.
Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

// Creating my kinematics object using the wheel locations.
MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
// MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
//   m_kinematics,
//   m_gyro.getRotation2d(),
//   new MecanumDriveWheelPositions(
//     m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
//     m_rearLeftEncoder.getDistance(), m_rearRightEncoder.getDistance()
//   ),
//   new Pose2d(0, 0, new Rotation2d()) //START POSE
// );


//   @Override
//   public void periodic() {
//       // Get my wheel positions
//   var wheelPositions = new MecanumDriveWheelPositions(
//     m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
//     m_rearLeftEncoder.getDistance(), m_rearRightEncoder.getDistance());

//   // Get the rotation of the robot from the gyro.
//   var gyroAngle = m_gyro.getRotation2d();

//   // Update the pose
//   var m_pose = m_odometry.update(gyroAngle, wheelPositions);
//   }
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_MecanumDrive.setMaxOutput(maxOutput);
  }

  @SuppressWarnings("ParameterName")
  public void MecanumDrive(double xSpeed, double ySpeed, double rot) {
      m_MecanumDrive.driveCartesian(xSpeed, ySpeed, rot);
  }

  // public void resetEncoders() {
  //   m_frontLeftEncoder.reset();
  //   m_rearLeftEncoder.reset();
  //   m_frontRightEncoder.reset();
  //   m_rearRightEncoder.reset();
  // }

  public void zeroHeading() {
    m_gyro.reset();
  }

  
    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /** Initializes Shuffleboard motor objects */
  public void initShuffleboard(){
    //plot mecananum drive to shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Robot");
    tab.addDouble("Front Left Speed", () -> m_frontLeftMotor.get())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(0, 0);
    tab.addDouble("Front Right Speed", () -> m_frontRightMotor.get())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(1, 0);
    tab.addDouble("Rear Left Speed", () -> m_rearLeftMotor.get())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(0, 1);
    tab.addDouble("Rear Right Speed", () -> m_rearRightMotor.get())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withPosition(1, 1);
  }

  /** PID CONTROLLERS */
  // public PIDController controller = new PIDController(DriveConstants.k_driveLinKp, DriveConstants.k_driveLinKi, DriveConstants.k_driveLinKd);
  // public PIDController turnController = new PIDController(DriveConstants.k_driveAngKp, DriveConstants.k_driveAngKi, DriveConstants.k_driveAngKd);

}