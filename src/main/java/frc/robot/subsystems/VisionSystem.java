// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

@Logged(strategy = Strategy.OPT_OUT)
public class VisionSystem extends SubsystemBase {

  // TODO: move to constants.java
  final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  PhotonCamera topRightCamera;
  PhotonCamera bottomCamera;
  AprilTagFieldLayout tagLayout;
  PhotonPoseEstimator poseEstimatorTopRight;
  PhotonPoseEstimator poseEstimatorBottom;
  EstimatedRobotPose currentEstimatedPose = new EstimatedRobotPose(new Pose3d(), getTimeStamp(), new ArrayList<>(), poseStrategy);
  PhotonPipelineResult latestResult;

  private Field2d vision_field = new Field2d();
  private LEDSystem prettyLights;

  public Pose2d desiredPose = new Pose2d();
  public Field2d desiredField = new Field2d();

  boolean blueAlliance = true;

  /** Creates a new vision. */
  public VisionSystem(LEDSystem prettyLights) {
    this.prettyLights = prettyLights;
    topRightCamera = new PhotonCamera(VisionConstants.TOPRIGHT_CAMERA_NICKNAME);
    bottomCamera = new PhotonCamera(VisionConstants.BOTTOM_CAMERA_NICKNAME);
    PortForwarder.add(5800, "photonvision.local", 5800);

    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
    } catch (IOException exception) {
      topRightCamera.close();
      bottomCamera.close();
      throw new RuntimeException(exception);
    }

    poseEstimatorTopRight = new PhotonPoseEstimator(tagLayout, poseStrategy,
        VisionConstants.TOPRIGHT_CAMERA_PLACEMENT); // TODO: decide which pose strategy to use
    poseEstimatorBottom = new PhotonPoseEstimator(tagLayout, poseStrategy,
        VisionConstants.BOTTOM_CAMERA_PLACEMENT); // TODO: decide which pose strategy to use

    Shuffleboard.getTab("vision").add("vision based field", vision_field).withWidget(BuiltInWidgets.kField);
    Shuffleboard.getTab("vision").add("Desired Position", desiredField).withWidget(BuiltInWidgets.kField);

    periodic();
  }

  public void updateAlliance() {
    var alliance = DriverStation.getAlliance();
    blueAlliance = alliance.get() == DriverStation.Alliance.Blue;
  }

  @NotLogged
  public PhotonTrackedTarget findBestTargetReef() {
    if (latestResult.hasTargets()) {
      var results = new ArrayList<PhotonTrackedTarget>(latestResult.getTargets());
      results.sort((target1, target2) -> Double.compare(target1.area, target2.area));

      return results.get(results.size() - 1);
    }
    
    return null;
  }

  /*
   * Gets the yaw of the target in degrees (positive right).
   */

  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  /*
   * Get the pitch of the target in degrees (positive up).
   */

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  /*
   * Get the area of the target (how much of the camera feed the bounding box
   * takes up) as a percent (0-100).
   */

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  /*
   * Get the skew of the target (the skew of the target in degrees
   * counter-clockwise positive).
   */

  public double getTargetSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  // /* TODO: Added in docs but not in library
  // * Get 4 corners of the minimum bounding box rectagle
  // */

  // public List<TargetCorner> getTargetCorners(PhotonTrackedTarget target) {
  // return target.getCorners();
  // }

  /*
   * Get the ID of the detected fiducial marker.
   */

  public int getTargetID(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  /*
   *
   */

  public double getPoseAmbiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  // /* TODO: Included in docs but not in library
  // * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
  // * object/fiducial tag space (X forward, Y left, Z up) with the lowest
  // * reprojection error.
  // */

  // public Transform2d getCameraToTarget(PhotonTrackedTarget target) {
  // return target.getCameraToTarget();
  // }

  /*
   * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
   * object/fiducial tag space (X forward, Y left, Z up) with the lowest
   * reprojection error.
   */

  public Transform3d getBestPathToTarget(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  /*
   * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
   * object/fiducial tag space (X forward, Y left, Z up) with the highest
   * reprojection error.
   */

  public Transform3d getOtherPathToTarget(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  
  // @NotLogged
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseTopRight() {
  //   // TODO: if re-added, then implement like `getEstimatedGlobalPose`.
  // }

  // @NotLogged
  // public EstimatedRobotPose getEstimatedGlobalPoseBottom() {
  //   return getEstimatedGlobalPose();
  // }

  @NotLogged
  public double getTimeStamp() {
    if (getEstimatedGlobalPose() != null) {
      return getEstimatedGlobalPose().timestampSeconds;  
    }
    else {
      return 0.0;
    }
  }

  /*
   * Estimate the position of the robot relitive to the field.
   */
  @NotLogged
  public EstimatedRobotPose getEstimatedGlobalPose() {
    return currentEstimatedPose;
  }

  public Pose2d getEstimatedGlobalPose2d() {
    return getEstimatedGlobalPose().estimatedPose.toPose2d();
  }

  /*
   * Get the position of the tag relitive to the field.
   */

  public Optional<Pose3d> getTagPose(int targetID) {
    return tagLayout.getTagPose(targetID); // TODO: make this return a non-optional Pose3d
  }

  /*
   * Get the angle of the tag on the field in radians
   */
  public double getTagFieldAngle(int targetID) {
    return tagLayout.getTagPose(targetID).get().getRotation().getAngle();
  }

  /*
   * Calculate your robot’s Pose3d on the field using the pose of the AprilTag
   * relative to the camera, pose of the AprilTag relative to the field, and the
   * transform from the camera to the origin of the robot.
   */
  // TODO: Only use function if
  // (tagLayout.getTagPose(target.getFiducialId()).isPresent())

  public Pose3d getFieldRelativePose(Pose3d tagPose, Transform3d cameraToTarget) {
    return PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose, VisionConstants.TOPRIGHT_CAMERA_PLACEMENT);
  }

  // TODO: Define 2d version of camera placement to use this function
  // public Pose2d getFieldRelativePose( Pose2d tagPose, Transform2d
  // cameraToTarget) {
  // return PhotonUtils.estimateFieldToRobot(cameraToTarget, tagPose,
  // VisionConstants.LEFT_CAMERA_PLACEMENT);
  // }

  /*
   * Calculate the distance to the target based on the hieght of the camera off of
   * the ground, the hieght of the target off of the ground, the camera’s pitch,
   * and the pitch to the target.
   */

  public double getDistanceToTarget(double targetHeight, double cameraPitch,
      double targetPitch) {
    return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.TOPRIGHT_CAMERA_PLACEMENT.getY(), targetHeight,
        cameraPitch,
        Units.degreesToRadians(targetPitch)); // TODO: convert cameraPitch to use a constant
  }

  /*
   * Calculate the distance between two poses. This is useful when using
   * AprilTags, given that there may not be an AprilTag directly on the target.
   */

  public double getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getDistanceToPose(robotPose, targetPose);
  }

  /*
   * Calculate translation to the target based on the distance to the target and
   * angle to the target (yaw).
   */

  public Translation2d getTranslationToTarget(double distanceToTarget, double targetYaw) {
    return PhotonUtils.estimateCameraToTargetTranslation(distanceToTarget, Rotation2d.fromDegrees(-targetYaw));
  }

  /*
   * Calculate the Rotation2d between your robot and a target. This is useful when
   * turning towards an arbitrary target on the field.
   */

  public Rotation2d getYawToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getYawToPose(robotPose, targetPose);
  }

  /*
   * Toggle driver mode on or off. Driver mode is an unfiltered/normal view of the
   * camera to be used while driving the robot.
   */

  public void driverModeToggle(boolean toggleOn) {
    topRightCamera.setDriverMode(toggleOn);
  }

  /*
   * Set the pipeline used by the camera.
   */

  public void setPipelineIndex(int index) {
    topRightCamera.setPipelineIndex(index);
  }

  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
    if (pose != null) {
      desiredField.setRobotPose(pose);
    } else {
      desiredField.setRobotPose(new Pose2d());
    }
  }

  // /* TODO: Docs say we don't care
  // * Get the latency of the pipeline in miliseconds.
  // */

  // public double getPipelineLatency() {
  // var visionFrame = leftCamera.getLatestResult();
  // return visionFrame.getLatencyMillis();
  // }

  /*
   * Set the mode of the camera LED(s).
   */

  public void setLED(VisionLEDMode LEDMode) {
    topRightCamera.setLED(LEDMode);
  }

  boolean hasTag = false;

  void updatePhotonPipelineResult(PhotonPipelineResult pipelineResult) {
    latestResult = pipelineResult;
    var newPoseBottom = poseEstimatorBottom.update(pipelineResult);

    if (newPoseBottom.isPresent()) {
      currentEstimatedPose = newPoseBottom.get();
    }

    hasTag = pipelineResult.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    List<PhotonPipelineResult> pipelineResults = bottomCamera.getAllUnreadResults();
    
    for (var pipelineResult: pipelineResults) {
      updatePhotonPipelineResult(pipelineResult);
    }

    // TODO: verify that periodic doesn't run faster than photonvision, which could lead to this variable being toggled
    // true/false rapidly in succession.
    // if (pipelineResults.isEmpty()) {
    //   hasTag = false;
    // }

    var pose2d = getEstimatedGlobalPose().estimatedPose.toPose2d();
    vision_field.setRobotPose(pose2d);
    // TODO: might not be necessary?
    SignalLogger.writeDoubleArray("visionOdometry",
        new double[] { pose2d.getX(), pose2d.getY(), pose2d.getRotation().getDegrees() });

    LEDPattern pattern = LEDPattern.solid(Color.kGreen);

    // if (hasTag) {
    //   if (!prettyLights.hasTopPattern("Has Tag")) {
    //     prettyLights.addTopPattern("Has Tag", 10, pattern);
    //   }

    // } else {
    //   prettyLights.removeTopPattern("Has Tag");
    // }
  }
}
