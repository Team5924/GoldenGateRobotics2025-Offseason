/*
 * VisionIOCamera.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.visionnew;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LimelightHelpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionIOCamera implements VisionIO {

    public class Camera extends PhotonCamera {
        public final Transform3d robotToCamera;
        public final PhotonPoseEstimator poseEstimator;

        public Camera(String initialName, Transform3d robotToCamera) {
            super(initialName);
            this.robotToCamera = robotToCamera;
            poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(Constants.FIELD_TYPE),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        }
    }

    private Camera[] cameras;

    // TODO: add camera disconnected alerts sometime later

    public VisionIOCamera() {
        cameras = new Camera[]{
            new Camera(Constants.FRONT_RIGHT_NAME, Constants.FRONT_RIGHT_TRANSFORM),
            new Camera(Constants.FRONT_LEFT_NAME, Constants.FRONT_LEFT_TRANSFORM),
            new Camera(Constants.BACK_RIGHT_NAME, Constants.BACK_RIGHT_TRANSFORM),
            new Camera(Constants.BACK_LEFT_NAME, Constants.BACK_LEFT_TRANSFORM)
        };
    }

    public boolean allCamerasConnected() {
        for (Camera camera : cameras) {
            if (!camera.isConnected()) return false;
        }
        return true;
    }

    private boolean removeResult(PhotonPipelineResult result) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (Constants.BARGE_TAG_IDS.contains(target.getFiducialId())) return true;
        }
        return false;
    }

    private ArrayList<PhotonPipelineResult> filterResults(List<PhotonPipelineResult> results) {
        ArrayList<PhotonPipelineResult> updatedResults = new ArrayList<PhotonPipelineResult>(results);
        for (int i = 0; i < results.size(); ++i) {
            if (removeResult(results.get(i)))
            updatedResults.remove(i);
        }
        return updatedResults;
    }

    private ArrayList<EstimatedRobotPose> getEstimatedPoses(Camera camera, ArrayList<PhotonPipelineResult> results) {
        ArrayList<EstimatedRobotPose> poses = new ArrayList<EstimatedRobotPose>();
        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> pose = camera.poseEstimator.update(result);
            if (pose.isPresent()) poses.add(pose.get());
        }

        return poses;
    }

    private boolean isInsideField(Translation2d position) {
        return position.getX() < -Constants.FIELD_BORDER_MARGIN
              || position.getX()
                  > Constants.FIELD_LENGTH + Constants.FIELD_BORDER_MARGIN
              || position.getY() < -Constants.FIELD_BORDER_MARGIN
              || position.getY()
                  > Constants.FIELD_WIDTH + Constants.FIELD_BORDER_MARGIN;
    }

    public void periodicAddMeasurements(SwerveDrivePoseEstimator estimator) {
        for (Camera camera : cameras) {
            ArrayList<PhotonPipelineResult> results = filterResults(camera.getAllUnreadResults());

            ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(camera, results);
            for (EstimatedRobotPose pose : estimatedPoses) {
                if (isInsideField(pose.estimatedPose.getTranslation().toTranslation2d())
                    && (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                    (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                            && pose.targetsUsed.get(0).poseAmbiguity < 0.05
                            && pose.targetsUsed.get(0).area > 0.25
                            )
                    )
                ) {
                    estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                }
            }
        }
    }
}
