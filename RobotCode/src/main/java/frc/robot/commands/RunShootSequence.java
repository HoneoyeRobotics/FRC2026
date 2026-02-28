// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallHandlingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShootSequence extends Command {
  private final BallHandlingSubsystem ballHandlingSubsystem;

  /** Creates a new RunShootSequence. */
  public RunShootSequence(BallHandlingSubsystem ballHandlingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandlingSubsystem);
    this.ballHandlingSubsystem = ballHandlingSubsystem;
    Preferences.setDouble("ShootRPM", Preferences.getDouble("ShootRPM", 2900));
  }

  private final Timer currentTimer = new Timer();
  private double TargetRPM = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: get distance from the center
    double distance = 3;
    // calculate target rpm by distance
    // TargetRPM = 763 + (217*distance) + (-6.91*distance * distance); //based on
    // 65deg
    TargetRPM = 841 + (405 * distance) + (-17.8 * distance * distance); // based on 80 deg

    currentTimer.restart();
    TargetRPM = Preferences.getDouble("ShootRPM", 2900);
    ballHandlingSubsystem.setShooterVelocity(TargetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run the shooter
    // BallHandlingSubsystem.runShooter(0.55);

    ballHandlingSubsystem.runPickup(0.66);
    if (ballHandlingSubsystem.shooterAtVelocity()) {
      ballHandlingSubsystem.moveBottomFeeder(1);
      ballHandlingSubsystem.moveColumnFeeder(.5);
      ballHandlingSubsystem.moveColumnKicker(.5);
      SmartDashboard.putString("Loop Part", "shooting");
    }
    // if the shooter is no longer at velocity, and has been shooting for 1 second,
    // then stop.
    else {
      ballHandlingSubsystem.moveBottomFeeder(1);
      ballHandlingSubsystem.moveColumnFeeder(0.3);
      ballHandlingSubsystem.moveColumnKicker(0);
      SmartDashboard.putString("Loop Part", "resetting....");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    ballHandlingSubsystem.moveBottomFeeder(0);
    ballHandlingSubsystem.moveColumnFeeder(0);
    ballHandlingSubsystem.moveColumnKicker(0);
    ballHandlingSubsystem.setShooterVelocity(0);
    ballHandlingSubsystem.runPickup(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
