// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Components.DriveSubSys.DriveSubSys_Constants;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSubSys_Rotate2Heading extends PIDCommand {
  /** Creates a new DriveSubSys_Rotate2Heading Command
  *  Turns to robot to the specified angle.
  *
  * @param targetAngleDegrees The angle to turn to
  * @param driveSubSys        The drive subsystem to use
  */

  public DriveSubSys_Rotate2Heading(double targetAngleDegrees, DriveSubSys_Old driveSubSys) {
    super(
        // The controller that the command will use
        new PIDController(2, 0.5, 0),
        // This should return the measurement
        () -> driveSubSys.getHeading().getRadians(),
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees*Math.PI/180,
        // This uses the output
        output -> driveSubSys.Drive(0, 0, output, false, false, false));    
        
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
    return getController().atSetpoint();
  }
}
