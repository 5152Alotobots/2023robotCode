/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.MecanumDrive.Subsys_MecanumDrive;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /**
   ***** Library Components 
   */

  // ---- Power Distribution
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // ---- NavXGyro
  //public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  // ---- Pigeon2
  public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();

  // ---- LimeLight
  private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // ---- Drive Subsystem (Swerve)
  public final Subsys_MecanumDrive driveSubSys = new Subsys_MecanumDrive();

  /*
  ***** Charged Up Componentes
  */
 
  // ---- Driver Station
  public final SubSys_DriverStation driverStation = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /*
  ***** Auto Commands
  */
  /*

  private final Command m_Auto_PathPlanner_Test_Cmd =
      new DriveSubSys_PathPlanner_Test_Cmd(driveSubSys);
    
  private final Command m_Auto_PP_FollowTraj_Cmd =
      new DriveSubSys_PP_FollowTraj_Cmd("New New Path",driveSubSys);

  private final Command ihopethisworks =
      new DriveSubSys_PathPlanner_Test_Cmd(driveSubSys);
  */
  
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
   
    /**
    ****** Control System Components
    */

    // ---- Drive Subsystem Default Command
    driveSubSys.setDefaultCommand(new ());

      
      
      // new Cmd_SubSys_DriveTrain_JoysticDefault(
            //   driveSubSys,
            //   () -> driverStation.DriveFwdAxis(),
            //   () -> driverStation.DriveStrAxis(),
            //   () -> driverStation.DriveRotAxis(),
            //   true,
            //   () -> driverStation.RotateLeftPt(),
              //  () -> driverStation.RotateRightPt())
         

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
    

    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
   private void configureButtonBindings() {
 
    // Gyro Reset Command Button
    // driverStation.GyroResetButton.onTrue(
        // new InstantCommand(driveSubSys::zeroGyro, driveSubSys));
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //m_DriveSubSys.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    
    //return m_BasicAutoCmd;
    return m_chooser.getSelected();
  }
}
