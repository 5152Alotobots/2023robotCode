// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSubSys_Rotate2HeadingProfiled extends ProfiledPIDCommand {
  /** Creates a new DriveSubSys_Rotate2HeadingProfiled. */
  public DriveSubSys_Rotate2HeadingProfiled(double targetAngleDegrees, DriveSubSys_Old driveSubSys) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            2, 0.5, 0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(.5, .5)),
        // This should return the measurement
        () -> driveSubSys.getHeading().getRadians(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetAngleDegrees*Math.PI/180,0),
        //targetAngleDegrees*Math.PI/180,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          driveSubSys.Drive(0, 0, output, false, false, false);
          SmartDashboard.putNumber("Profile", setpoint.position);
        });

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveSubSys);
      // Configure additional PID options by calling `getController` here.
      getController().enableContinuousInput(-Math.PI, Math.PI);
      getController().setTolerance(1*Math.PI/180, 10*Math.PI/180);
      getController().setIntegratorRange(-.5, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
