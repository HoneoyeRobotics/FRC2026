// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.video.TrackerGOTURN;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {

  private Drivetrain m_Drivetrain;

  private DoubleSupplier m_turnSupplier;
  private DoubleSupplier m_forwardSupplier;
  private BooleanSupplier m_fastSupplier;
  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier turnSupplier, BooleanSupplier fastSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Drivetrain = drivetrain;
    m_forwardSupplier = forwardSupplier;
    m_turnSupplier = turnSupplier;
    m_fastSupplier = fastSupplier;

    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward  = m_forwardSupplier.getAsDouble();
    double turn = m_turnSupplier.getAsDouble();

    if(m_fastSupplier.getAsBoolean() == false)
    {
      forward = forward / 2;
      turn = turn / 2;
    }

    m_Drivetrain.drive(forward, turn );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_Drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
