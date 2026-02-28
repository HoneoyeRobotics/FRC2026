// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import static frc.robot.Constants.Vision.*;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
  }

  private PhotonCamera LeftAprilTagCamera = new PhotonCamera("LeftAprilTagCamera");
  private PhotonCamera RightAprilTagCamera = new PhotonCamera("RightAprilTagCamera");
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  //positive yaw to the left
  public static final Transform3d LeftRobotToCam = new Transform3d(new Translation3d(0, 0.0381, 0.6096),
      new Rotation3d(0, 0, 0.436332));
  public static final Transform3d RightRobotToCam = new Transform3d(new Translation3d(0, -0.0381, 0.6096),
      new Rotation3d(0, 0, -0.436332));
  private PhotonPoseEstimator LeftPoseEstimator = new PhotonPoseEstimator(kTagLayout, LeftRobotToCam);

  private PhotonPoseEstimator RightPoseEstimator = new PhotonPoseEstimator(kTagLayout, RightRobotToCam);

  private Matrix<N3, N1> curStdDevs;
  private final EstimateConsumer estConsumer;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // var latestResult = LeftAprilTagCamera.getLatestResult();
    // SmartDashboard.putBoolean("Has Target", latestResult.hasTargets());
    // boolean multiTagPresent = false;
    // var results = LeftAprilTagCamera.getAllUnreadResults();
    // for (var result : results) {
    // var multiTagResult = result.getMultiTagResult();
    // if (multiTagResult.isPresent()) {
    // multiTagPresent = true;
    // var fieldToCamera = multiTagResult.get().estimatedPose.best;

    // SmartDashboard.putNumber("F2C - X", fieldToCamera.getX());
    // SmartDashboard.putNumber("F2C - Y", fieldToCamera.getY());
    // SmartDashboard.putNumber("F2C - Z", fieldToCamera.getZ());
    // SmartDashboard.putNumber("F2C - Angle",
    // fieldToCamera.getRotation().getAngle());
    // SmartDashboard.putNumber("F2C - AngleD",
    // Math.toDegrees(fieldToCamera.getRotation().getAngle()));
    // double x = fieldToCamera.getX();
    // double y = fieldToCamera.getY();

    // double angle = Math.toDegrees(Math.atan((4.035 - y) / (4.626 - x)) * -1);
    // angleToGoal = angle;
    // SmartDashboard.putNumber("Target Angle", angleToGoal);
    // }
    // }
    // SmartDashboard.putBoolean("Multitag Is present", multiTagPresent);

    UpdatePoseFromCamera(LeftAprilTagCamera, LeftPoseEstimator);
    UpdatePoseFromCamera(RightAprilTagCamera, RightPoseEstimator);

  }

  private void UpdatePoseFromCamera(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEst, result.getTargets(), photonEstimator);

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator photonEstimator) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

}