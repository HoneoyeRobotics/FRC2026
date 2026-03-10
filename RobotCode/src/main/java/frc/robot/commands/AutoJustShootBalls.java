// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallHandlingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoJustShootBalls extends SequentialCommandGroup {
  /** Creates a new AutoJustShootBalls. */
  public AutoJustShootBalls(DriveSubsystem driveSubsystem, BallHandlingSubsystem ballHandlingSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new TogglePickupSolenoid(ballHandlingSubsystem),
        new DriveRobot(driveSubsystem, 0.5, 0, 0).withTimeout(1),
        new RotateToGoal(driveSubsystem, visionSubsystem).withTimeout(1.5),
        new RunShootSequence(ballHandlingSubsystem, driveSubsystem).withTimeout(8)

    );
  }
}
