/*
 * VisionIO.java
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

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.MegatagPoseEstimate;

public interface VisionIO {
  @AutoLog
  public static class VisionIOCameraInputs {
    public boolean seesTarget;
    public FiducialObservation fiducials;

    public MegatagPoseEstimate megatag2PoseEstimateFrontLeft = null;

    public Pose2d megatag2PoseEstimatePose2d = null;
    public int megatag2PoseEstimateTagCount = 0;
    public double megatag2PoseEstimateAvgTagArea = 0;
    public double lowestTagAmbiguity = 1;

    public double pipelineLatencySeconds = 0.0;
    public double captureLatencySeconds = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOCameraInputs inputs[]) {}
}
