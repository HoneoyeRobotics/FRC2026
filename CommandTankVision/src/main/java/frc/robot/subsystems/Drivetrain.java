// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {


  private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(11);
  private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(10);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  /** Creates a new Drivertrain. */
  public Drivetrain() {
    m_rightMotor.setInverted(true);
  }


  public void drive(double forward, double turn){
        m_robotDrive.arcadeDrive(forward, turn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
