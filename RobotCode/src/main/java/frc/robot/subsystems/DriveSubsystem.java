// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.Perspective;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degree;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(2);
  private final SwerveDrivePoseEstimator poseEstimator;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {

    return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState());
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

  }

  public void driveSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    poseEstimator.update(getGyroYaw(), getModulePositions());

    var EstimatedPosition = poseEstimator.getEstimatedPosition();
    odometry.update(
        EstimatedPosition.getRotation(),
        // Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()), // IMUAxis.kZ)),

        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    double estDistance = getDistanceFromGoal();
    DecimalFormat df = new DecimalFormat("#.###");
    SmartDashboard.putString("Est Distance", df.format(estDistance));
    SmartDashboard.putNumber("PoseEst X", EstimatedPosition.getX());
    SmartDashboard.putNumber("PoseEst Y", EstimatedPosition.getY());
    SmartDashboard.putString("PoseEst Rot", df.format(EstimatedPosition.getRotation().getDegrees()));

    SmartDashboard.putNumber("Heading", getCompassHeading());

  }

  public double getDistanceFromGoal() {

    var EstimatedPosition = poseEstimator.getEstimatedPosition();
    double currX = EstimatedPosition.getX();
    double currY = EstimatedPosition.getY();

    double goalX = 4.622;
    double goalY = 4.025;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // 16.54m total field length, so go to the other and and go back.
      goalX = 16.54 - goalX;
    }

    return Math.sqrt(Math.pow((currX - goalX), 2) + Math.pow((currY - goalY), 2));
  }

  /** Raw gyro yaw (this may not match the field heading!). */
  public Rotation2d getGyroYaw() {
    return gyro.getRotation2d();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {

    var EstimatedPose = poseEstimator.getEstimatedPosition();

    return EstimatedPose;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /**
   * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean GoFast) {

    if (GoFast == false) {
      xSpeed /= 4;
      ySpeed /= 4;
      rot /= 4;
    }

    if (fieldRelative && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      // currentRotation = currentRotation.rotateBy(Rotation2d.kPi);
      xSpeed *= -1;
      ySpeed *= -1;
    }

    SmartDashboard.putBoolean("GoFast", GoFast);
    SmartDashboard.putNumber("Drive xSpeed", xSpeed);
    SmartDashboard.putNumber("Drive ySpeed", ySpeed);
    SmartDashboard.putNumber("Drive zRot", rot);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    Rotation2d currentRotation = poseEstimator.getEstimatedPosition().getRotation();

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, currentRotation)
            // ? Perspective.OPERATOR.toPerspectiveSpeeds( new
            // ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
            // currentRotation)
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.setDefaultBoolean("Swap Red Rotation", false);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  public Rotation2d getHeadingRotation2d() {
    return getPose().getRotation();
  }

  public double getCompassHeading() {
    return MathUtil.inputModulus(getHeading(), -180, 180);
  }

  public double getEstimatedHeading() {
    var EstimatedPose = poseEstimator.getEstimatedPosition();

    return EstimatedPose.getRotation().getDegrees();
  }

  public double getOffsetToGoal() {
    var EstimatedPose = poseEstimator.getEstimatedPosition();

    double targetAngle = getAngleToGoal();
    double currentAngle = EstimatedPose.getRotation().getDegrees();

    return targetAngle - currentAngle;
  }

  public double getAngleToGoal() {

    var EstimatedPose = poseEstimator.getEstimatedPosition();
    double goalX = 4.622;
    double goalY = 4.025;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // 16.54m total field length, so go to the other and and go back.
      goalX = 16.54 - goalX;
    }

    SmartDashboard.putNumber("GoalY", goalY);
    SmartDashboard.putNumber("Goal X", goalX);

    double y = EstimatedPose.getY();
    double x = EstimatedPose.getX();

    DecimalFormat df = new DecimalFormat("#.###");
    double angleRad = Math.atan((goalY - y) / (goalX - x)) * -1;
    double angleDeg = Math.toDegrees(angleRad);
        SmartDashboard.putString("AngleToGoal pre", df.format(angleDeg));

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // && SmartDashboard.getBoolean("Swap Red Rotation", false)) {
      if (angleDeg < 0)
        angleDeg = -180 - angleDeg;
      if (angleDeg == 0)
        angleDeg = 180;
      if (angleDeg > 0)
        angleDeg = 180- angleDeg ;
    } 
    SmartDashboard.putString("AngleToGoal", df.format(angleDeg));
    return angleDeg;

  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
