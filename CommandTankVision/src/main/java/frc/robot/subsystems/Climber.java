// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

  //private  climberMotor;

    private SparkMax climber = new SparkMax(26, MotorType.kBrushless);

  

  public void MoveClimber(double speed) {
    climber.set(speed);
  }

  /** Creates a new Climber. */
  public Climber() {}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
