// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BallHandlingSubsystem extends SubsystemBase {

  private SparkMax PickupLeftMotor;
  private SparkMax PickupRightMotor;
  private SparkMax BottomFeederMotor;
  private SparkMax ColumnFeederMotor;
  private SparkMax ColumnKickerMotor;
  private SparkMax ShooterMotor;
  private Compressor Compressor;
  private DoubleSolenoid PickupSolenoid;
  private boolean PickupOut = false;

  // private SparkClosedLoopController ShooterPIDController;
  // private RelativeEncoder ShooterEncoder;

  /** Creates a new BallHandlingSubsystem. */
  public BallHandlingSubsystem() {
    PickupLeftMotor = new SparkMax(MotorConstants.PickupLeftCanID, MotorType.kBrushless);
    PickupRightMotor = new SparkMax(MotorConstants.PickupRightCanID, MotorType.kBrushless);

    BottomFeederMotor = new SparkMax(MotorConstants.BottomFeederCanID, MotorType.kBrushless);
    ColumnFeederMotor = new SparkMax(MotorConstants.ColumnFeederCanID, MotorType.kBrushless);
    ColumnKickerMotor = new SparkMax(MotorConstants.ColumnKickerCanID, MotorType.kBrushless);
    ShooterMotor = new SparkMax(MotorConstants.ShooterCanID, MotorType.kBrushless);
    SparkMaxConfig ShooterMotorConfig = new SparkMaxConfig();
    ShooterMotorConfig.idleMode(IdleMode.kCoast);
    ShooterMotorConfig.inverted(true);
    ShooterMotor.configure(ShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig KickerMotorConfig = new SparkMaxConfig();
    KickerMotorConfig.idleMode(IdleMode.kBrake);
    KickerMotorConfig.inverted(true);
    ColumnKickerMotor.configure(KickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig BottomFeederMotorConfig = new SparkMaxConfig();
    BottomFeederMotorConfig.idleMode(IdleMode.kBrake);
    BottomFeederMotorConfig.inverted(true);
    BottomFeederMotor.configure(BottomFeederMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // ShooterPIDController = ShooterMotor.getClosedLoopController();
    // ShooterEncoder = ShooterMotor.getEncoder();
    // ShooterMotorConfig = new SparkMaxConfig();
    // ShooterMotorConfig.encoder.velocityConversionFactor(1);

    // ShooterMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .p(Constants.ShooterConstants.proportialPIDConstant)
    // .i(Constants.ShooterConstan3ts.integralPIDConstant)
    // .d(Constants.ShooterConstants.derivativePIDConstant)
    // .outputRange(Constants.ShooterConstants.minPIDOutput,
    // Constants.ShooterConstants.maxPIDOutput)
    // .feedForward.kV(12.0 / 5767);

    Compressor = new Compressor(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM);
    PickupSolenoid = new DoubleSolenoid(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM,
        Constants.PneumaticsConstants.PickupPneumaticCylinderForwardChannel,
        Constants.PneumaticsConstants.PickupPneumaticCylinderReverseChannel);
    PickupSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void togglePickupSolenoid() {
    if (PickupOut == true) {
      PickupSolenoid.set(DoubleSolenoid.Value.kReverse);
      PickupOut = false;
    } else {
      PickupSolenoid.set(DoubleSolenoid.Value.kForward);
      PickupOut = true;
    }
  }

  public boolean shooterAtVelocity() {

    if (ShooterMotor.getEncoder().getVelocity() >= 2750)
      return true;
    else
      return false;
    // return false;// ShooterPIDController.isAtSetpoint();
  }

  public void toggleShooterVelocity(double velocity) {
    // if(ShooterPIDController.getSetpoint() == 0)
    // ShooterPIDController.setSetpoint(velocity, ControlType.kVelocity);
    // else
    // ShooterPIDController.setSetpoint(0, ControlType.kVelocity);
  }

  public void setShooterVelocity(double velocity) {
    // ShooterPIDController.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void runPickup(double speed) {
    PickupLeftMotor.set(speed);
    PickupRightMotor.set(speed * -1);
  }

  public void runShooter(double speed) {
    ShooterMotor.set(speed);
  }

  public void moveBottomFeeder(double speed) {
    BottomFeederMotor.set(speed);
  }

  public void moveColumnFeeder(double speed) {
    ColumnFeederMotor.set(speed);
  }

  public void moveColumnKicker(double speed) {
    ColumnKickerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", ShooterMotor.getEncoder().getVelocity());
  }
}
