/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.ColorSensor.SubSys_ColorSensor;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.MecanumDrive.SubSys_MecanumDrive;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.Hand.Cmd.Cmd_HandWithSensor;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
// import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.ChargedUp.MecanumDrive.Cmd.Cmd_MecanumDriveDefault;
import frc.robot.ChargedUp.DistanceSensor.SubSys_DistanceSensor;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  //public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  XboxController m_driverController = new XboxController(0); 

  public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();
 
  public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  public final SubSys_DriverStation driverStation = new SubSys_DriverStation();

  // ---- Hand
  public final SubSys_Hand handSubSys = new SubSys_Hand();
  
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();

    mecanumDriveSubSys.setDefaultCommand(
      new Cmd_MecanumDriveDefault(
        mecanumDriveSubSys, 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getRightX())
      ); 

    handSubSys.setDefaultCommand(
      new Cmd_HandWithSensor(handSubSys, colorSubSys, distanceSubsys, () -> driverStation.HandSensorBtn())
    );
  }
  public void periodic() {

  }    
    /*driveSubSys
      .setDefaultCommand(new Cmd_SubSys_DriveTrain_JoysticDefault(
        driveSubSys,
        () -> driverStation.DriveFwdAxis(),
        () -> driverStation.DriveStrAxis(),
        () -> driverStation.DriveRotAxis(),
        true,
        () -> driverStation.RotateLeftPt(),
        () -> driverStation.RotateRightPt()));
    */
    
    
    // Sendable Chooser
    //m_chooser.setDefaultOption("Auto_BasicRevHighGoalRev_Cmd", m_Auto_BasicRevHighGoalRev_Cmd);
    //m_chooser.addOption("Auto_BasicRevLowGoalRev", m_Auto_BasicRevLowGoalRev_Cmd);
    //m_chooser.addOption("Auto_AS_RevHighGoalRev_Cmd", m_Auto_AS_RevHighGoalRev_Cmd);
    //m_chooser.addOption("Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd", m_Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd);
    //m_chooser.addOption("Auto_PathPlanner_Test_Cmd", m_Auto_PathPlanner_Test_Cmd);
    //m_chooser.addOption("Auto_PP_FollowTraj_Cmd", m_Auto_PP_FollowTraj_Cmd);
    //m_chooser.addOption("goodluck", ihopethisworks);
    //m_chooser.setDefaultOption("Drive4Distance", m_Drive4Distance_Cmd);
    //m_chooser.addOption("BasicAutoLowWait", m_BasicAutoLowWaitCmd);
    //m_chooser.addOption("BasicAutoHigh", m_BasicAutoHighCmd);
    //m_chooser.addOption("BasicAutoHighExtraBalls", m_BasicAutoHighExtraBallsCmd);
    //m_chooser.addOption("HighshotAuto", m_LeftCenterHigh_Cmd);
    
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driverStation.GyroResetButton.onTrue(new InstantCommand(mecanumDriveSubSys::zeroGyro, mecanumDriveSubSys));
    driverStation.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStation.CloseHandButton.onTrue(new InstantCommand(handSubSys::CloseHand, handSubSys));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
