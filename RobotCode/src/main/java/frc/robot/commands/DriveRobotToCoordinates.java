// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveRobotToCoordinates extends Command {
  private final DriveSubsystem driveSubsystem;
  private final double xSpeed;
  private final double ySpeed;
  private final double zRotation;

  private final double xCoord;
  private final double yCoord;

  /** Creates a new CenterOnAprilTag. */
  public DriveRobotToCoordinates(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed, double zRotation,
      double xCoord, double yCoord) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.xCoord = xCoord;
    this.yCoord = yCoord;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
      xCoord = 16.54 - xCoord;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(xSpeed, ySpeed, zRotation, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var pose = driveSubsystem.getEstimatedPose();

    // if current position is further than we want to go on the X, we are done.

    // if we are red, pose > xcoord
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {

      if (xSpeed > 0) {
        SmartDashboard.putString("DriveToCoordinates", pose.getX() + " > " + xCoord);
        return pose.getX() > xCoord;
      } else {
        SmartDashboard.putString("DriveToCoordinates", pose.getX() + " <>> " + xCoord);
        return pose.getX() < xCoord;
      }
    } else
    // if we are blue, pose < xcoord
    {
      if (xSpeed > 0) {
        SmartDashboard.putString("DriveToCoordinates", pose.getX() + " < " + xCoord);
        return pose.getX() < xCoord;
      } else {
        SmartDashboard.putString("DriveToCoordinates", pose.getX() + " < " + xCoord);
        return pose.getX() > xCoord;
      }
    }

  }
}
