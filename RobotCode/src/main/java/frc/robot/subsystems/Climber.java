// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    climber.getEncoder().setPosition(0.0);
  }

  private SparkMax climber = new SparkMax(Constants.MotorConstants.ClimberMotorCanID, MotorType.kBrushless);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climber Encoder", climber.getEncoder().getPosition());
  }

  public void MoveClimber(double speed) {

    if (climber.getEncoder().getPosition() >= 51 && speed > 0)
      climber.set(0);
    else
      climber.set(speed);

  }

}
