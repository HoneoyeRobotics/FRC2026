// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final DriveSubsystem driveSubsystem;

  private final CommandXboxController driverController;
  private final CommandJoystick coDriverJoystick;

  /** Creates a new CenterOnAprilTag. */
  public TeleopDrive(DriveSubsystem driveSubsystem, CommandXboxController driverController,
      CommandJoystick coDriverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    this.coDriverJoystick = coDriverJoystick;
    addRequirements(driveSubsystem);
  }

  private PIDController yPidController = new PIDController(0.03, 0.001, 0.001);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yPidController.reset();
  }

  private final double VISION_TURN_kP = 0.01;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
    double zRot = -MathUtil.applyDeadband(
        driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband);

    double RightxSpeed = -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.kDriveDeadband);
    double RightySpeed = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

    boolean fieldRelative = true;

    // if the left stick is all zero, but the right stick is not, then use the robot
    // relative drive.
    if (xSpeed == 0 && ySpeed == 0 && (RightxSpeed != 0 || RightySpeed != 0)) {
      fieldRelative = false;
      xSpeed = RightxSpeed;
      ySpeed = RightySpeed;
    }

    // if you are pushing on the left stick, modify based ont he limelight.
    if (driverController.leftBumper().getAsBoolean() == true || coDriverJoystick.button(3).getAsBoolean() == true) {
      fieldRelative = false;
      xSpeed = RightxSpeed;
      ySpeed = RightySpeed;

      double targetYaw = 0.0; // this is the yaw we want to be at

      targetYaw = driveSubsystem.getEstimatedHeading() -
      (driveSubsystem.getAngleToGoal() * -1);

      zRot = -1.0 * targetYaw * VISION_TURN_kP * Constants.DriveConstants.kMaxAngularSpeed;

    }

    // normal driving;
    driveSubsystem.drive(
        xSpeed,
        ySpeed,
        zRot,
        fieldRelative, driverController.rightBumper().getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
