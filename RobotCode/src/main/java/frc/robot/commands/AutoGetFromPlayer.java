// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallHandlingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGetFromPlayer extends SequentialCommandGroup {
  /** Creates a new AutoJustShootBalls. */
  public AutoGetFromPlayer(DriveSubsystem driveSubsystem, BallHandlingSubsystem ballHandlingSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveRobot(driveSubsystem, -0.5, 0, 0).withTimeout(2),
        new TogglePickupSolenoid(ballHandlingSubsystem),
        new ParallelDeadlineGroup(
            // new DriveRobotToCoordinates(driveSubsystem, 0.5, 0, 0, 0.8962,0),
            new SequentialCommandGroup(
                new DriveRobotToCoordinates(driveSubsystem, -0.5, 0, 0, 0.6962, 0, false).withTimeout(6),
                new WaitCommand(4))),

        new DriveRobotToCoordinates(driveSubsystem, 0.5, 0, 0, 2.4, 0, true).withTimeout(2),
        new RotateToGoal(driveSubsystem, visionSubsystem).withTimeout(1.5),
        new DriveRobot(driveSubsystem, 0.5, 0, 0).withTimeout(1),
        new RunShootSequence(ballHandlingSubsystem, driveSubsystem).withTimeout(8)

    );
  }
}
