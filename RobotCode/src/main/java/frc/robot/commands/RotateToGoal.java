// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToGoal extends Command {
  /** Creates a new RotateTogoal. */
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  public RotateToGoal(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(driveSubsystem);
  }
    private PIDController zPidController = new PIDController(0.03, 0.001, 0.001);

  private double rotationSetpoint = 9999;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationSetpoint = m_visionSubsystem.getAngleToGoal() * -1;
    zPidController = new PIDController(0.03, 0.001, 0.001);
    SmartDashboard.putNumber("Rotate Setpoint", rotationSetpoint);
    zPidController.setSetpoint(rotationSetpoint);
    zPidController.setIntegratorRange(-0.5, 0.5);
    zPidController.enableContinuousInput(-180, 180);
    zPidController.setTolerance(0.1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_driveSubsystem.getCompassHeading();

        
    double rotatePower = zPidController.calculate(currentAngle);    

    rotatePower = MathUtil.clamp(rotatePower, -0.3, 0.3);
    SmartDashboard.putNumber("Rotate Power",rotatePower);
    m_driveSubsystem.drive(
        0.0,
        0.0,
        rotatePower,
        false,
        false
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_driveSubsystem.drive(
        0.0,
        0.0,
        0.0,
        false,
        false
    );}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

return false;
  }
}
