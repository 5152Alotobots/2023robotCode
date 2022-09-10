/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_DriveDistanceTrajectory_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_DriveVariableTrajectory2_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_DriveVariableTrajectory_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_JoysticDefault_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_Range2LimeLightTarget_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_ResetOdometry_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_Rotate2LimeLightTarget_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_SeekRotate2LimeLightTarget_Cmd;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.pathplanner_cmds.DriveSubSys_PathPlanner_Test_Cmd;
import frc.robot.Components.GyroSubSys.Pigeon.PigeonGyroSubSys;
import frc.robot.Components.Vision.LimeLightSubSys;
import frc.robot.Components.Vision.TargetInterpolation;
import frc.robot.Constants.Field;
import frc.robot.Constants.RobotSettings;
import frc.robot.Constants.Field.StartingPos;
import frc.robot.RapidReact.Climber.ClimberInteraction;
import frc.robot.RapidReact.Climber.ClimberSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds.ClimberLift_Default_Joystick_Cmd;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds.ClimberLift_NoLoadSlot0_Pos_Cmd;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorCmds.ClimberRotator_Default_Joystick_Cmd;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorCmds.ClimberRotator_Pos_Cmd;
import frc.robot.RapidReact.DriverStation.DriverStationSubSys;
import frc.robot.RapidReact.Intake.IntakeInteraction;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmCmds.IntakeArm_Default_Joystick_Cmd;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmCmds.IntakeArm_PosHold_Cmd;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmCmds.IntakeArm_Pos_Cmd;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutSubSys;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutCmds.IntakeInNOut_Default_Joystick_Cmd;
import frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersSubSys;
import frc.robot.RapidReact.Intake.IntakeTriggers.IntakeTriggersCmds.IntakeTriggers_Default_Joystick_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutoShoot_Cmd;
import frc.robot.RapidReact.RapidReactCommands.BasicAutoArmHighPosExtraBall_Cmd;
import frc.robot.RapidReact.RapidReactCommands.BasicAutoArmHighPos_Cmd;
import frc.robot.RapidReact.RapidReactCommands.BasicAutoArmLowPosWait_Cmd;
import frc.robot.RapidReact.RapidReactCommands.BasicAutoArmLowPos_Cmd;
import frc.robot.RapidReact.RapidReactCommands.BasicAuto_Cmd;
import frc.robot.RapidReact.RapidReactCommands.LeftCenterHigh_Cmd;
import frc.robot.RapidReact.RapidReactCommands.LeftFrontLow_Cmd;
import frc.robot.RapidReact.RapidReactCommands.T13_B2_High_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_AS_FwdXtrBallRevWallRotAutoFwdHighGoal_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_AS_FwdXtrBallRotAutoFwdHighGoal_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_AS_FwdXtrBallRotFwdHighGoal_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_AS_RevHighGoalRev_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_BasicRevHighGoalRev_Cmd;
import frc.robot.RapidReact.RapidReactCommands.AutonomousCmds.Auto_BasicRevLowGoalRev_Cmd;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_Cmd;

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
   ***** Components - General Reusable Subsystems
   */

  /**
   ****** Control System Components
   */

  // ---- Power Distribution
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // ---- NavXGyro
  //public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  // ---- Pigeon2
  public final PigeonGyroSubSys m_PigeonGyroSubSys = new PigeonGyroSubSys();

  // ---- LimeLight
  private final LimeLightSubSys m_LimeLightSubSys = new LimeLightSubSys();

  // ---- Drive Subsystem (Swerve)
  public final DriveSubSys_Old m_DriveSubSys = new DriveSubSys_Old(m_PigeonGyroSubSys);

  /*
  ***** Rapid React Components
  */
  // Helper Classes
  private final IntakeInteraction m_IntakeInteraction = new IntakeInteraction();
  private final ClimberInteraction m_ClimberInteraction = new ClimberInteraction();
  
  // ---- Driver Station
  private final DriverStationSubSys m_DriverStation = new DriverStationSubSys(
    m_IntakeInteraction);

  // ---- Intake
  // ------ IntakeInNOut
  private final IntakeInNOutSubSys m_IntakeInNOutSubSys = new IntakeInNOutSubSys(
    m_IntakeInteraction); 

  // ------ IntakeArm
  private final IntakeArmSubSys m_IntakeArmSubSys = new IntakeArmSubSys(
    m_IntakeInteraction);

  // ------ IntakeTriggers
  private final IntakeTriggersSubSys m_IntakeTriggersSubSys = new IntakeTriggersSubSys(
    m_IntakeInteraction);

  // ---- Climber
  private final ClimberSubSys m_ClimberSubSys = new ClimberSubSys();

  // ------ Climber Rotator
  private final ClimberRotatorSubSys m_ClimberRotatorSubSys = new ClimberRotatorSubSys(
    m_ClimberInteraction);

  // ------ Climber Lift
  private final ClimberLiftSubSys m_ClimberLiftSubSys = new ClimberLiftSubSys(
    m_ClimberInteraction);

  /**
   ***** Infinate Recharge Components
   */

  // ---- Driver Station
  //private final DriverStationSubSys m_DriverStation = new DriverStationSubSys();

  // ---- Intake
  //private final IntakeSubSys m_IntakeSubSys = new IntakeSubSys();

  // ---- Revolver
  //private final RevolverSubSys m_RevolverSubSys = new RevolverSubSys();

  // ---- Shooter
  // private final ShooterSubSys m_ShooterSubSys = new ShooterSubSys();
  // ------ Shooter Muzzle
  //private final ShooterMuzzleSubSubSys m_ShooterMuzzleSubSubSys = new ShooterMuzzleSubSubSys();
  // ------ Shooter Slide
  //private final ShooterSlideSubSubSys m_ShooterSlideSubSubSys = new ShooterSlideSubSubSys();
  // ------ Shooter Wheels
  //private final ShooterWheelsSubSubSys m_ShooterWheelsSubSubSys = new ShooterWheelsSubSubSys();

  // ShooterTurret
  // private final ShooterTurretSubSubSys m_ShooterTurretSubSubSys = new
  // ShooterTurretSubSubSys();

  // Periscope
  // PeriscopeLift
  // private final PeriscopeLiftSubSubSys m_PeriscopeLiftSubSubSys = new
  // PeriscopeLiftSubSubSys();

  // TrolleyPeriscope
  // private final TrolleyPeriscopeSubSys m_TrolleyPeriscopeSubSys = new
  // TrolleyPeriscopeSubSys();

  // Lift
  // private final LiftSubSys m_LiftSubSys = new LiftSubSys();
  // private final LiftScissorLiftSubSubSys m_LiftScissorLiftSubSubSys = new
  // LiftScissorLiftSubSubSys();
  // private final LiftWinchSubSubSys m_LiftWinchSubSubSys = new
  // LiftWinchSubSubSys();
  // private final LiftLockSubSubSys m_LiftLockSubSubSys = new
  // LiftLockSubSubSys();
  // Trolly Periscope Color Sensor
  // private final PeriscopeColorSensorSubSubSys m_PeriscopeColorSensorSubSubSys =
  // new PeriscopeColorSensorSubSubSys();
  // |AUTONOMOUS COMMANDS|

  // private final Command m_MuzzleRotateCmd = new
  // ShooterMuzzleRotateCmd(m_ShooterMuzzleSubSubSys,m_LimeLightSubSys.ty/35);
  
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /*
  ***** Auto Commands
  */

  private final Command m_Auto_BasicRevHighGoalRev_Cmd =
    new Auto_BasicRevHighGoalRev_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);  

  private final Command m_Auto_BasicRevLowGoalRev_Cmd =
    new Auto_BasicRevLowGoalRev_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);

  private final Command m_Auto_AS_RevHighGoalRev_Cmd =
    new Auto_AS_RevHighGoalRev_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys,
      m_LimeLightSubSys);

  private final Command m_Auto_AS_FwdXtrBallRotFwdHighGoal_Cmd =
    new Auto_AS_FwdXtrBallRotFwdHighGoal_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys,
      m_LimeLightSubSys);

  private final Command m_Auto_AS_FwdXtrBallRotAutoFwdHighGoal_Cmd =
    new Auto_AS_FwdXtrBallRotAutoFwdHighGoal_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys,
      m_LimeLightSubSys);

  private final Command m_Auto_AS_FwdXtrBallRevWallRotAutoFwdHighGoal_Cmd =
    new Auto_AS_FwdXtrBallRevWallRotAutoFwdHighGoal_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys,
      m_LimeLightSubSys);

  private final Command m_Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd =
    new Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys,
      m_LimeLightSubSys);

  /* Obsolete
  private final Command m_BasicAutoCmd =
    new BasicAuto_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);
  

  private final Command m_BasicAutoLowCmd =
    new BasicAutoArmLowPos_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);

  private final Command m_BasicAutoLowWaitCmd =
      new BasicAutoArmLowPosWait_Cmd(
        m_DriveSubSys,
        m_IntakeArmSubSys,
        m_IntakeInNOutSubSys,
        m_IntakeTriggersSubSys);
        
  private final Command m_BasicAutoHighCmd =
        new BasicAutoArmHighPos_Cmd(
          m_DriveSubSys,
          m_IntakeArmSubSys,
          m_IntakeInNOutSubSys,
          m_IntakeTriggersSubSys);

  private final Command m_BasicAutoHighExtraBallsCmd =
    new BasicAutoArmHighPosExtraBall_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);

  private final Command m_LeftFrontLow_Cmd =
    new LeftFrontLow_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);

  private final Command m_LeftCenterHigh_Cmd =
    new LeftCenterHigh_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys,
      m_IntakeInNOutSubSys,
      m_IntakeTriggersSubSys);

  private final Command m_Drive4Distance_Cmd =
    new SequentialCommandGroup(
      // Initialize
      new DriveSubSys_ResetOdometry_Cmd(
        m_DriveSubSys,
        StartingPos.kT13_Start),

      //new DriveSubSys_DriveDistanceTrajectory_Cmd(
      //  m_DriveSubSys,
      //  Units.feetToMeters(-3))

      
      new DriveSubSys_DriveVariableTrajectory2_Cmd(
        m_DriveSubSys, 
        StartingPos.kT13_Start,
        List.of(
          new Translation2d(
            Units.inchesToMeters(231),
            Units.inchesToMeters(108)
          )), 
        new Pose2d(
          Field.Tarmac.kT13,
          new Rotation2d(Units.degreesToRadians(33))), 
        false)
        
    );
   */
    
/*
  private final Command m_T13_B2_HighTrajBased_Cmd =
    new T13_B2_High_Cmd(
      m_DriveSubSys,
      m_IntakeArmSubSys, m_IntakeInNOutSubSys, m_IntakeTriggersSubSys);
        
  /*
  private final Command m_AutoClimbLiftCmd =
    new ClimberLiftManual_Cmd(m_ClimberLiftSubSys,
      () -> m_DriverStation.ClimberLiftPositiveBtn(),
      () -> m_DriverStation.ClimberLiftNegativeBtn());
*/

  // private final Command m_AutoDrive4Time_Cmd =
  // new DriveSubSys_Drive4Time_Cmd(m_DriveSubSys, 0.25, 0.0, 0.0, 1, false);

  // private final Command m_AutoDrive2Heading_Cmd =
  // new DriveSubSys_Rotate2Heading_Cmd(0.0, m_DriveSubSys);

  // private final Command m_AutoDrive2HeadingProfiled_Cmd =
  // new DriveSubSys_Rotate2HeadingProfiled_Cmd(0, m_DriveSubSys);

  //private final Command m_AutoDriveTrajectory1_Cmd = new DriveSubSys_DriveTrajectory1_Cmd(m_DriveSubSys);

  //private final Command m_AutoDriveTrajectoryPathweaverInit_Cmd = new DriveSubSys_DriveTrajectoryPathweaverInit_Cmd(
  //    m_DriveSubSys);
  // FalconFXDriveTalonSRXSteerSwerveDriveAutoFwdDriveForTime(-.4,0,0, 5,
  // m_DriveSubSys);
  // private final Command m_AutoShootLowCommand = new
  // FalconFxDriveTalonSRXSteerSwerveDriveAutoShootLow(m_ShooterMuzzleSubSubSys,
  // m_RevolverSubSys, m_ShooterWheelsSubSubSys, m_ShooterSlideSubSubSys);

  // private final Command m_AutoShooterWheelSlideCommand = new
  // FalconFxDriveTalonSRXSteerSwerveDriveAutoShooterWheelSlideCmd(m_ShooterWheelsSubSubSys,
  // m_ShooterSlideSubSubSys);

  // private final Command m_ShooterSlideDelayCommand = new
  // ShooterSlideDelayCmd(m_ShooterSlideSubSubSys, Value.kReverse, 1);
  
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
    m_DriveSubSys
      .setDefaultCommand(new DriveSubSys_JoysticDefault_Cmd(
        m_DriveSubSys,
        () -> m_DriverStation.DriveFwdAxis(),
        () -> m_DriverStation.DriveStrAxis(),
        () -> m_DriverStation.DriveRotAxis(),
        true,
        () -> m_DriverStation.RotateLeftPt(),
        () -> m_DriverStation.RotateRightPt()));

    /*
    ***** Rapid React Components
    */

    // ---- Intake
    // ------ IntakeInNOut
    m_IntakeInNOutSubSys
      .setDefaultCommand(new IntakeInNOut_Default_Joystick_Cmd(
        m_IntakeInNOutSubSys,
        () -> m_DriverStation.IntakeInNOut_Intake(),
        () -> m_DriverStation.IntakeInNOut_ShortShotAxis(),
        () -> m_DriverStation.IntakeInNOut_LongShotAxis(),
        true));

    // ------ IntakeArm
    m_IntakeArmSubSys
      .setDefaultCommand(new IntakeArm_Default_Joystick_Cmd(
        m_IntakeArmSubSys,
        () -> m_DriverStation.IntakeArm_Axis()));

    // ------ Intake Triggers
    m_IntakeTriggersSubSys
      .setDefaultCommand(new IntakeTriggers_Default_Joystick_Cmd(
        m_IntakeTriggersSubSys,
        () -> m_DriverStation.IntakeLeftTriggerAxis(),
        () -> m_DriverStation.IntakeRightTriggerAxis()));

    // ---- Climber
    // ------ Climber Lift Default Command
    m_ClimberLiftSubSys
      .setDefaultCommand(new ClimberLift_Default_Joystick_Cmd(
        m_ClimberLiftSubSys,
        () -> m_DriverStation.ClimberLiftPositiveBtn(),
        () -> m_DriverStation.ClimberLiftNegativeBtn()));

    // ------ Climber Rotator Default Command
    m_ClimberRotatorSubSys
      .setDefaultCommand(new ClimberRotator_Default_Joystick_Cmd(
        m_ClimberRotatorSubSys, 
        () -> m_DriverStation.ClimberLiftRotatorPositiveBtn(),
        () -> m_DriverStation.ClimberLiftRotatorNegativeBtn()));
        
    

    // IntakeSubSys.setDefaultCommand
    //m_IntakeSubSys.setDefaultCommand(new Intake_DefaultCmd(
    //  m_IntakeSubSys,
    //  () -> m_DriverStation.IntakeButton(),
    //  () -> m_DriverStation.ShootButton(),
    //  () -> m_DriverStation.LiftAxis(),
    //  () -> m_DriverStation.LeftShootButton(),
    //  () -> m_DriverStation.RightShootButton()));

    // Revolver
    //m_RevolverSubSys.setDefaultCommand(new RevolverRotateButtonCmd(m_RevolverSubSys, 0.0));

    // Shooter

    // Shooter Muzzle
    //m_ShooterMuzzleSubSubSys.setDefaultCommand(
    //  new ShooterMuzzleRotateHoldPosCmd(m_ShooterMuzzleSubSubSys, () -> m_DriverStation.ShooterMuzzleRotateAxis()));

    // m_ShooterMuzzleSubSubSys.setDefaultCommand(new
    // ShooterMuzzleJoystickPositionTrackingCmd(m_ShooterMuzzleSubSubSys,
    // m_RevolverSubSys, () -> m_DriverStation.ShooterMuzzleRotateAxis()));

    // Shooter Slide
    //m_ShooterSlideSubSubSys
    //    .setDefaultCommand(new ShooterSlideModeCmd(m_ShooterSlideSubSubSys, DoubleSolenoid.Value.kOff));

    // Shooter Wheels
    //m_ShooterWheelsSubSubSys.setDefaultCommand(new ShooterWheelsSpdCmd(m_ShooterWheelsSubSubSys, 0));

    // Shooter Turret
    /*
     * m_ShooterTurretSubSubSys.setDefaultCommand( new ShooterTurretSpdCmd(
     * m_ShooterTurretSubSubSys, 0.0));
     */

    /*
     * m_ShooterTurretSubSubSys.setDefaultCommand( new ShooterTurretPosCmd(
     * m_ShooterTurretSubSubSys, 0.0));
     */

    // Periscope
    // PeriscopeLift
    // m_PeriscopeLiftSubSubSys.setDefaultCommand(new
    // PeriscopeLiftModeCmd(m_PeriscopeLiftSubSubSys, DoubleSolenoid.Value.kOff));

    // TrolleyPeriscope
    // m_TrolleyPeriscopeSubSys.setDefaultCommand(new
    // TrolleyPeriscopeSpdCmd(m_TrolleyPeriscopeSubSys, 0.0));

    // m_PeriscopeColorSensorSubSubSys.setDefaultCommand(new
    // PeriscopeStage1Cmd(m_PeriscopeColorSensorSubSubSys, m_TrolleyPeriscopeSubSys,
    // false));

    // Lift
    // LiftScissorLift
    // m_LiftScissorLiftSubSubSys.setDefaultCommand(new
    // LiftScissorLiftCmd(m_LiftScissorLiftSubSubSys, false));

    // LiftWinch
    // m_LiftWinchSubSubSys.setDefaultCommand(new
    // LiftWinchSpdCmd(m_LiftWinchSubSubSys, 0));

    // Sendable Chooser
    // m_chooser.setDefaultOption("AutoDrive4Time", m_AutoDrive4Time_Cmd);
    // m_chooser.setDefaultOption("AutoDriveTrajectory1",
    // m_AutoDriveTrajectory1_Cmd);
    //m_chooser.setDefaultOption("AutoDriveTrajejctoryPathweaver", m_AutoDriveTrajectoryPathweaverInit_Cmd);
    //m_chooser.addOption("AutoDriveTrajectoryPathweaver", m_AutoDriveTrajectoryPathweaverInit_Cmd);
    // m_chooser.addOption("AutoDrive2HeadingProfiled",
    // m_AutoDrive2HeadingProfiled_Cmd);
    // m_chooser.addOption("AutoDriveTrajectory1", m_AutoDriveTrajectory1_Cmd);
    // m_chooser.addOption("SlideDelayCmd", m_ShooterSlideDelayCommand);

    // m_chooser.addOption("AutoDriveFwd", m_AutoDriveFwdCommand);

    //Shuffleboard.getTab("Auto").add(m_chooser).withSize(8, 4).withPosition(0, 0);

    // Sendable Chooser
    m_chooser.setDefaultOption("Auto_AS_RevHighGoalRev_Cmd", m_Auto_AS_RevHighGoalRev_Cmd);
    m_chooser.addOption("Auto_AS_FwdXtrBallRotFwdHighGoal_Cmd", m_Auto_AS_FwdXtrBallRotFwdHighGoal_Cmd);
    m_chooser.addOption("Auto_AS_FwdXtrBallRotAutoFwdHighGoal_Cmd", m_Auto_AS_FwdXtrBallRotAutoFwdHighGoal_Cmd);
    m_chooser.addOption("Auto_AS_FwdXtrBallRevWallRotAutoFwdHighGoal_Cmd", m_Auto_AS_FwdXtrBallRevWallRotAutoFwdHighGoal_Cmd);
    m_chooser.addOption("Auto_BasicRevHighGoalRev_Cmd", m_Auto_BasicRevHighGoalRev_Cmd);
    m_chooser.addOption("Auto_BasicRevLowGoalRev", m_Auto_BasicRevLowGoalRev_Cmd);

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

    // IntakeArmHighShootPos
    m_DriverStation.IntakeArmHighPosButton.whenPressed(
      new IntakeArm_PosHold_Cmd(
        m_IntakeArmSubSys,
        RobotSettings.Shooting.kArmHighGoalAngle,
        () -> m_DriverStation.IntakeArm_Axis()));

    // IntakeArmLowShootPos
    m_DriverStation.IntakeArmLowPosButton.whenPressed(
      new IntakeArm_PosHold_Cmd(
        m_IntakeArmSubSys,
        RobotSettings.Shooting.kArmLowGoalAngle,
        () -> m_DriverStation.IntakeArm_Axis()));

    // IntakeArmHighShootPos
    m_DriverStation.IntakeArmDownPosButton.whenPressed(
      new IntakeArm_PosHold_Cmd(
        m_IntakeArmSubSys,
        RobotSettings.Intake.kArmIntakeAngle,
        () -> m_DriverStation.IntakeArm_Axis()));    
        
    // Extend up to Rung2 Pos Command Button  
    m_DriverStation.LiftFwdPosButton.whileHeld(
      new ParallelCommandGroup(
        // Lower Intake
        new IntakeArm_Pos_Cmd(
          m_IntakeArmSubSys,
          RobotSettings.Intake.kArmIntakeAngle),

        // Extend up to Rung 2
        new ClimberLift_NoLoadSlot0_Pos_Cmd(
          m_ClimberLiftSubSys, 
          TalonFXControlMode.Position,
          RobotSettings.Climbing.kExtend2Rung2Pos)));
        
    /*
    // Lift Rev Pos Command Button  
      m_DriverStation.LiftRevPosButton.whenPressed(
        new ClimberLift_Pos_Cmd(
          m_ClimberLiftSubSys, 
          TalonFXControlMode.MotionMagic,
          500)
          .withTimeout(7));
    */

    // Climb Enable Command Button
      m_DriverStation.ClimbEnableButton.whileHeld(
        new Endgame3_Cmd(
          m_ClimberSubSys,
          m_ClimberLiftSubSys,
          m_ClimberRotatorSubSys,
          m_IntakeArmSubSys));

    // Climb Reset Command Button
      m_DriverStation.ClimbResetButton.whenPressed(
        new InstantCommand(m_ClimberSubSys::resetClimbState, m_ClimberSubSys));  
  
    // Gyro Reset Command Button
    m_DriverStation.GyroResetButton.whenPressed(
        new InstantCommand(m_DriveSubSys::zeroGyro, m_DriveSubSys));
    
    // Limelight Auto Rotate Command Button
    m_DriverStation.LimeLightAutoRotate2HubButton.whileHeld(
      new DriveSubSys_SeekRotate2LimeLightTarget_Cmd(
        m_DriveSubSys, 
        () -> m_DriverStation.DriveFwdAxis(),
        () -> m_DriverStation.DriveStrAxis(),
        true,
        m_LimeLightSubSys));
    
    // Limelight Auto Range Command Button
    m_DriverStation.LimeLightAutoShootButton.whileHeld(
      //new ShooterAutoTarget_Cmd(
      //  m_DriveSubSys, 
      //  m_IntakeArmSubSys,
      //  m_IntakeInNOutSubSys, 
      //  m_LimeLightSubSys));

      new AutoShoot_Cmd(
        m_DriveSubSys, 
        m_IntakeArmSubSys, 
        m_IntakeInNOutSubSys, 
        m_LimeLightSubSys, 
        () -> m_DriverStation.DriveFwdAxis(),
        () -> m_DriverStation.DriveStrAxis())
    );
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
