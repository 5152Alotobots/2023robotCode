// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.MK4i_FalconFalcon.MK4i_FalconFalcon_Module;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

public class SubSys_SwerveDrive extends SubsystemBase {
    /** Creates a new SubSys_SwerveDrive. */
 
    public MK4i_FalconFalcon_Module[] swerveModules;
    public SwerveDriveOdometry swerveOdometry;
    private SwerveModulePosition[] swerveModulePositionsInit;
    private SubSys_PigeonGyro gyroSubSys;
    // Rotate About Point
    private Translation2d rotationPt;
    private boolean rotateLeftPtCmd_prev;
    private boolean rotateRightPtCmd_prev;

    public SubSys_SwerveDrive(SubSys_PigeonGyro gyroSubSys) {
        this.gyroSubSys = gyroSubSys;
        
        this.swerveModulePositionsInit = new SwerveModulePosition[]{
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0)),
            new SwerveModulePosition(0.0, new Rotation2d(0.0))
        };

        // Odometry class for tracking robot pose
        this.swerveOdometry = new SwerveDriveOdometry(
            SubSys_SwerveDrive_Constants.swerveKinematics,
            getHeading(),
            this.swerveModulePositionsInit);

        this.swerveModules = new MK4i_FalconFalcon_Module[] {
            new MK4i_FalconFalcon_Module("FL", SubSys_SwerveDrive_Constants.FL_constants),
            new MK4i_FalconFalcon_Module("FR", SubSys_SwerveDrive_Constants.FR_constants),
            new MK4i_FalconFalcon_Module("BL", SubSys_SwerveDrive_Constants.BL_constants),
            new MK4i_FalconFalcon_Module("BR", SubSys_SwerveDrive_Constants.BR_constants)
        };

        this.rotationPt = new Translation2d(0,0);

        this.rotateLeftPtCmd_prev = false;
        this.rotateRightPtCmd_prev = false;
    }

    /** Drive Command
     * 
     * @param translation       Translation2d X and Y Robot Velocities in m/s
     * @param rotation          Double Rotational Velocity in rads/s
     * @param fieldRelative     Boolean Field Relative
     * @param isOpenLoop        Boolean Open Loop Velocity Control
     * @param rotateLeftPtCmd   Boolean Rotate around Left Point Cmd
     * @param rotateRightPtCmd  Boolean Rotate around Right Point Cmd
     */
    public void drive(
        Translation2d translation,
        double rotation,
        boolean fieldRelative,
        boolean isOpenLoop,
        boolean rotateLeftPtCmd,
        boolean rotateRightPtCmd) {

        // Determine Rotation Point
        rotationPt = SwerveRotatePointLogic.calcRotationPt(
            rotateLeftPtCmd, 
            rotateRightPtCmd,
            rotateLeftPtCmd_prev,
            rotateRightPtCmd_prev,
            fieldRelative,
            getHeading().getDegrees(),
            translation.getX());

        // Set Status of RotatePt Buttons for next loop
        rotateLeftPtCmd_prev = rotateLeftPtCmd;
        rotateRightPtCmd_prev = rotateRightPtCmd;

        // Calculate the Swerve Module States
        SwerveModuleState[] swerveModuleStates =
            SubSys_SwerveDrive_Constants.swerveKinematics.toSwerveModuleStates(
                (fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)),
                rotationPt);
            SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                Constants.RobotSettings.DriveTrain.DriveTrainMaxSpd);

        // Set Swerve Modules to Calculated States
        swerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);  // FL
        swerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);  // FR
        swerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);  // BL
        swerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);  // BR
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates,
          Constants.RobotSettings.DriveTrain.DriveTrainMaxSpd);

          swerveModules[0].setDesiredState(desiredStates[0], false);  // FL
          swerveModules[1].setDesiredState(desiredStates[1], false);  // FR
          swerveModules[2].setDesiredState(desiredStates[2], false);  // BL
          swerveModules[3].setDesiredState(desiredStates[3], false);  // BR
          
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(
            getHeading(), 
            getSwerveModulePositions(), 
            pose);
    }

    /** getServeModulePositions
    * 
    * @return SwerveModulePosition[4] Positions of the Swerve Modules 0- FrontLeft, 1- FrontRight, 2- BackLeft, 3- BackRight
    */
    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

        swerveModulePositions[0] = swerveModules[0].getPosition();
        swerveModulePositions[1] = swerveModules[1].getPosition();
        swerveModulePositions[2] = swerveModules[2].getPosition();
        swerveModulePositions[3] = swerveModules[3].getPosition();

        return swerveModulePositions;
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = swerveModules[0].getState();  // FL
        states[1] = swerveModules[1].getState();  // FR
        states[2] = swerveModules[2].getState();  // BL
        states[3] = swerveModules[3].getState();  // BR

        return states;
    }

    public void zeroGyro(){
        gyroSubSys.setYaw(0.0);  
    }

    public Rotation2d getHeading() {
        return gyroSubSys.getGyroRotation2d();
    }

    public SwerveDriveKinematics getSwerveDriveKinematics(){
        return SubSys_SwerveDrive_Constants.swerveKinematics;
    }

    @Override
    public void periodic(){
        SwerveModulePosition[] swerveModulePositions = getSwerveModulePositions();

        swerveOdometry.update(
            getHeading(), 
            swerveModulePositions);  

        SmartDashboard.putNumber("FL_SteerSensor_AbsPos", swerveModules[0].getSteerSensorAbsolutePos());
        SmartDashboard.putNumber("FL_SteerSensor_Pos", swerveModules[0].getSteerSensorPos());
        SmartDashboard.putNumber("FL_SteerMotor_Pos", swerveModules[0].getSteerMotorPos());
    }
}
