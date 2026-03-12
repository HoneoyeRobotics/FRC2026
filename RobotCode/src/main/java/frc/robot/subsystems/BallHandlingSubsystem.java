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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import java.lang.annotation.Target;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BallHandlingSubsystem extends SubsystemBase {

  private SparkMax PickupLeftMotor;
  private SparkMax PickupRightMotor;
  private SparkMax BottomFeederMotor;
  private SparkMax ColumnFeederMotor;
  private SparkMax ColumnKickerMotor;
  private SparkMax ShooterLeftMotor;
  private SparkMax ShooterRightMotor;
  private Compressor Compressor;
  private DoubleSolenoid PickupSolenoid;
  private boolean PickupOut = false;

  private PIDController ShooterPIDController = new PIDController(0.1, 0, 0);
  private RelativeEncoder ShooterEncoder;
  private boolean ShooterPIDEnabled = false;

  private boolean PreviouslyAtSetpoint = false;

  private SparkClosedLoopController pidController;
  private double FeedForward = 0.59;

  /** Creates a new BallHandlingSubsystem. */
  public BallHandlingSubsystem() {
    PickupLeftMotor = new SparkMax(MotorConstants.PickupLeftCanID, MotorType.kBrushless);
    PickupRightMotor = new SparkMax(MotorConstants.PickupRightCanID, MotorType.kBrushless);

    BottomFeederMotor = new SparkMax(MotorConstants.BottomFeederCanID, MotorType.kBrushless);
    ColumnFeederMotor = new SparkMax(MotorConstants.ColumnFeederCanID, MotorType.kBrushless);
    ColumnKickerMotor = new SparkMax(MotorConstants.ColumnKickerCanID, MotorType.kBrushless);
    ShooterLeftMotor = new SparkMax(MotorConstants.ShooterLeftCanID, MotorType.kBrushless);
    ShooterRightMotor = new SparkMax(MotorConstants.ShooterRightCanID, MotorType.kBrushless);
    
    SparkMaxConfig ShooterLeftMotorConfig = new SparkMaxConfig();
    ShooterLeftMotorConfig.idleMode(IdleMode.kCoast);
    ShooterLeftMotorConfig.inverted(true);
    ShooterLeftMotorConfig.encoder.uvwAverageDepth(1).uvwMeasurementPeriod(10);
    ShooterLeftMotor.configure(ShooterLeftMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SparkMaxConfig ShooterRightMotorConfig = new SparkMaxConfig();
    ShooterRightMotorConfig.idleMode(IdleMode.kBrake);
    ShooterRightMotorConfig.inverted(false);
    ShooterRightMotorConfig.encoder.uvwAverageDepth(1).uvwMeasurementPeriod(10);
    ShooterRightMotor.configure(ShooterRightMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SparkMaxConfig KickerMotorConfig = new SparkMaxConfig();
    KickerMotorConfig.idleMode(IdleMode.kBrake);
    KickerMotorConfig.inverted(true);
    ColumnKickerMotor.configure(KickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig ColumnFeederMotorConfig = new SparkMaxConfig();
    ColumnFeederMotorConfig.idleMode(IdleMode.kBrake);
    ColumnFeederMotorConfig.inverted(false);
    ColumnFeederMotor.configure(ColumnFeederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig BottomFeederMotorConfig = new SparkMaxConfig();
    BottomFeederMotorConfig.idleMode(IdleMode.kCoast);
    BottomFeederMotorConfig.inverted(true);
    BottomFeederMotor.configure(BottomFeederMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    pidController = ShooterLeftMotor.getClosedLoopController();
    ShooterEncoder = ShooterLeftMotor.getEncoder();

    // ShooterLeftMotorConfig = new SparkMaxConfig();
    // ShooterLeftMotorConfig.encoder.velocityConversionFactor(1);

    // ShooterLeftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .p(0.94)
    // .i(0)
    // .d(0)
    // .outputRange(Constants.ShooterConstants.minPIDOutput,
    // Constants.ShooterConstants.maxPIDOutput)
    // .feedForward.kV(0.38);//.kA(0.78);

    // ShooterLeftMotor.configure(ShooterLeftMotorConfig,
    // ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);

    Compressor = new Compressor(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM);
    PickupSolenoid = new DoubleSolenoid(Constants.PneumaticsConstants.CompressorCanID, PneumaticsModuleType.CTREPCM,
        Constants.PneumaticsConstants.PickupPneumaticCylinderForwardChannel,
        Constants.PneumaticsConstants.PickupPneumaticCylinderReverseChannel);
    // PickupSolenoid.set(DoubleSolenoid.Value.kReverse);
    PickupSolenoid.set(DoubleSolenoid.Value.kReverse);

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
    return ShooterPIDController.atSetpoint();
  }

  public Command setShooterVelocityCommand(double rpm) {
    return runOnce(() -> SetShooterControllerVelocity(rpm));
  }

  public void SetShooterControllerVelocity(double velocity) {
    pidController.setSetpoint(2900, ControlType.kVelocity);
  }

  public void setShooterVelocityByDistance(double distance) {

    double TargetRPM = 0;

    // THIS IS FROM OUR GOOGLE DOCUMENT
    TargetRPM = 132.23417 * distance * distance - 375.3763 * distance + 2820.15427;
    TargetRPM = TargetRPM *  1.015;

    if(distance < 3)
      TargetRPM = TargetRPM * 1.065;
    SmartDashboard.putNumber("Google RPM", TargetRPM);
    // this is from real math
    double shooterAngle = 85;
    double lossPct = 1 / 0.80;
    // TargetRPM = lossPct * 188 * (distance / Math.cos(shooterAngle))
    //     * Math.sqrt(9.81 / (2 * distance * Math.tan(shooterAngle) - 1.067));
    SmartDashboard.putNumber("Mathed RPM", TargetRPM);

    // our mathed ff
    //FeedForward = 0.3288484 * Math.pow(1.000196, distance);

    // real FF
    FeedForward = TargetRPM / 5676;// * 1.1;
    SmartDashboard.putNumber("Feed Forward", FeedForward);

    setShooterVelocity(TargetRPM);
  }

  public void setShooterVelocity(double velocity) {
    ShooterPIDController.reset();
    ShooterPIDController.setSetpoint(velocity);
    if (velocity == 0) {
      ShooterPIDEnabled = false;
      ShooterLeftMotor.set(0);
      ShooterRightMotor.set(0);
    } else {

      ShooterPIDController.reset();
      ShooterPIDController.setP(Preferences.getDouble("ShootP", 0.0002));
      ShooterPIDController.setI(Preferences.getDouble("ShootI", 0));
      ShooterPIDController.setD(Preferences.getDouble("ShootD", 0));
      ShooterPIDController.setTolerance(Preferences.getDouble("ShooterTolerance", 50));
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
    ShooterLeftMotor.set(speed);
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

    SmartDashboard.putNumber("Shooter RPM", ShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Setpoint", ShooterPIDController.getSetpoint());
    SmartDashboard.putBoolean("Shooter At Setpoint", shooterAtVelocity());
    SmartDashboard.putNumber("Shooter Tolerance", ShooterPIDController.getErrorTolerance());
    SmartDashboard.putBoolean("PID Enabled", ShooterPIDEnabled);
    if (ShooterPIDEnabled == true) {

      double ff = Preferences.getDouble("ShooterFF", 0.59);

      double speed = ShooterPIDController.calculate(ShooterEncoder.getVelocity()) + FeedForward;

      // if(speed < 0.1)
      // speed = 0.1;
      ShooterLeftMotor.set(speed);
      ShooterRightMotor.set(speed);
      SmartDashboard.putNumber("Shooter Speed", speed);
    } else {
      SmartDashboard.putNumber("Shooter Speed", 0.0);
    }
  }
}
