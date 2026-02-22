// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BallHandlingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterVelocity extends InstantCommand {
    private final BallHandlingSubsystem m_BallHandlingSubsystem;
private final double m_TargetVelocity;
  /** Creates a new ShootBalls. */
  public SetShooterVelocity(BallHandlingSubsystem ballHandlingSubsystem, double TargetVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(ballHandlingSubsystem);
    m_BallHandlingSubsystem = ballHandlingSubsystem;
    m_TargetVelocity = TargetVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_BallHandlingSubsystem.setShooterVelocity(m_TargetVelocity);
  }

}
