// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallHandlingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickupBalls extends Command {
  /** Creates a new PickupBalls. */
  private final BallHandlingSubsystem ballHandlingSubsystem;
  private final double speed;

  /***
   * This command runs the front pickup motors
   * 
   * @param ballHandlingSubsystem Required ball handling subsystem
   * @param speed                 Speed that the pickup runs at. Positive values
   *                              bring balls into the robot.
   */
  public PickupBalls(BallHandlingSubsystem ballHandlingSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(ballHandlingSubsystem);
    this.ballHandlingSubsystem = ballHandlingSubsystem;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballHandlingSubsystem.runPickup(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballHandlingSubsystem.runPickup(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
