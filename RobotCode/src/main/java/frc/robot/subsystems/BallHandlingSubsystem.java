// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
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
  private PIDController ShooterPIDController = new PIDController(0.1, 0, 0);
  private RelativeEncoder ShooterEncoder;
  private boolean ShooterPIDEnabled = false;

  private boolean PreviouslyAtSetpoint = false;

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
    // .i(Constants.ShooterConstants.integralPIDConstant)
    // .d(Constants.ShooterConstants.derivativePIDConstant)
    // .outputRange(Constants.ShooterConstants.minPIDOutput,
    // Constants.ShooterConstants.maxPIDOutput)
    // .feedForward.kV(12.0 / 5767);

    // ShooterMotor.configure(ShooterMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kNoPersistParameters);

    Compressor = new Compressor(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM);
    PickupSolenoid = new DoubleSolenoid(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM,
        Constants.PneumaticsConstants.PickupPneumaticCylinderForwardChannel,
        Constants.PneumaticsConstants.PickupPneumaticCylinderReverseChannel);
    //PickupSolenoid.set(DoubleSolenoid.Value.kReverse);
    PickupSolenoid.set(DoubleSolenoid.Value.kForward);

    Preferences.setDouble("ShooterTolerance", Preferences.getDouble("ShooterTolerance", 50));
    ShooterPIDController.setTolerance(Preferences.getDouble("ShooterTolerance", 50));
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

    if (ShooterPIDEnabled == false)
      return false;

    double setpoint = ShooterPIDController.getSetpoint();
    double velocity = ShooterMotor.getEncoder().getVelocity();
    double tolerance = Preferences.getDouble("ShooterTolerance", 50);
    double upperTolerance = tolerance;
    double lowerTolerance = tolerance;
    if (PreviouslyAtSetpoint == false)
      lowerTolerance = tolerance/4;
    else
      upperTolerance = tolerance * 2;

    if (velocity >= (setpoint - lowerTolerance) && (velocity <= (setpoint + upperTolerance))) {
      if (PreviouslyAtSetpoint == false) {
        ShooterPIDController.setTolerance(tolerance);
        PreviouslyAtSetpoint = true;
      }
      return true;
    } else
      return false;
  }

  public void setShooterVelocity(double velocity) {
    ShooterPIDController.reset();
    ShooterPIDController.setSetpoint(velocity);
    if (velocity == 0) {
      ShooterPIDEnabled = false;
      ShooterMotor.set(0);
    } else {

      ShooterPIDController.reset();
      ShooterPIDController.setP(Preferences.getDouble("ShootP", 0.0002));
      ShooterPIDController.setI(Preferences.getDouble("ShootI", 0));
      ShooterPIDController.setD(Preferences.getDouble("ShootD", 0));
      ShooterPIDController.setTolerance(0);
      PreviouslyAtSetpoint = false;
      ShooterPIDEnabled = true;
    }
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
    // // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter RPM", ShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Setpoint", ShooterPIDController.getSetpoint());
    SmartDashboard.putBoolean("Shooter At Setpoint", shooterAtVelocity());

    SmartDashboard.putBoolean("PID Enabled", ShooterPIDEnabled);
    if (ShooterPIDEnabled == true) {

      double ff = Preferences.getDouble("ShooterFF", 0.59);
      double speed = ShooterPIDController.calculate(ShooterMotor.getEncoder().getVelocity()) + ff;

      // if(speed < 0.1)
      //   speed = 0.1;
      ShooterMotor.set(speed);
      SmartDashboard.putNumber("Shooter Speed", speed);
    } else {
      SmartDashboard.putNumber("Shooter Speed", 0.0);
    }
  }
}
