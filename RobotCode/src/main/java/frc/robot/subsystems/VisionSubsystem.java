// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}
  private PhotonCamera camera = new PhotonCamera("LeftAprilTagCamera");



public double getAngleToGoal(){
//  var results = camera.getAllUnreadResults();
//     for (var result : results) {
//       var multiTagResult = result.getMultiTagResult();
//       if (multiTagResult.isPresent()) {
//           var fieldToCamera = multiTagResult.get().estimatedPose.best;        

//          double x = fieldToCamera.getX();
//         double y = fieldToCamera.getY();

//         double angle  = Math.atan((4.035-y)/ (4.626-x)) * -1;
//         return Math.toDegrees(angle);

//       }
//     }

//     return 9999;

  return angleToGoal;
}

private double angleToGoal = 9999;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
            var latestResult = camera.getLatestResult();


    SmartDashboard.putBoolean("Has Target",  latestResult.hasTargets());
    boolean multiTagPresent = false;
    var results = camera.getAllUnreadResults();
    for (var result : results) {
      var multiTagResult = result.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        multiTagPresent = true;
        var fieldToCamera = multiTagResult.get().estimatedPose.best;        

        SmartDashboard.putNumber("F2C - X", fieldToCamera.getX());
        SmartDashboard.putNumber("F2C - Y", fieldToCamera.getY());
        SmartDashboard.putNumber("F2C - Z", fieldToCamera.getZ());
        SmartDashboard.putNumber("F2C - Angle", fieldToCamera.getRotation().getAngle());
        SmartDashboard.putNumber("F2C - AngleD", Math.toDegrees(fieldToCamera.getRotation().getAngle()));
        double x = fieldToCamera.getX();
        double y = fieldToCamera.getY();

        double angle  = Math.toDegrees(Math.atan((4.035-y)/ (4.626-x)) * -1);
        angleToGoal = angle;
        SmartDashboard.putNumber("Target Angle",angleToGoal);
      }
    }
    SmartDashboard.putBoolean("Multitag Is present", multiTagPresent);

//   Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);

// double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);

//     Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
//   distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
// Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);

  }
}