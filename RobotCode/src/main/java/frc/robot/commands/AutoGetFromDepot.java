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
public class AutoGetFromDepot extends SequentialCommandGroup {
  /** Creates a new AutoJustShootBalls. */
  public AutoGetFromDepot(DriveSubsystem driveSubsystem, BallHandlingSubsystem ballHandlingSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new TogglePickupSolenoid(ballHandlingSubsystem),
        new DriveRobot(driveSubsystem, 0.5, 0, 0).withTimeout(1),
        new ParallelDeadlineGroup(        
            //new DriveRobotToCoordinates(driveSubsystem, 0.5, 0, 0, 0.8962,0),            
            new SequentialCommandGroup(
              new DriveRobotToCoordinates(driveSubsystem, 0.5, 0, 0, 0.6962,0),           
              new WaitCommand(1),
              new DriveRobot(driveSubsystem, 0, -0.2, 0).withTimeout(0.2),
              new DriveRobot(driveSubsystem, 0, 0.2, 0).withTimeout(0.2)
              ),
            new PickupBalls(ballHandlingSubsystem, 0.66)
        ),            
        new ParallelDeadlineGroup(
            new DriveRobotToCoordinates(driveSubsystem, -0.5, 0, 0, 1.4,0),           
            new PickupBalls(ballHandlingSubsystem, 0.66)

        ),
        new RotateToGoal(driveSubsystem, visionSubsystem).withTimeout(1.5),
        new DriveRobot(driveSubsystem, 0.5, 0, 0).withTimeout(0.5),        
        new RunShootSequence(ballHandlingSubsystem, driveSubsystem).withTimeout(8)

    );
  }
}
