/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *  STOLEN FROM TEAM 1710 BECAUSE OLD VISION GOT TOO CLUTTERED
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera aprilTagCameraBr;
    private final PhotonCamera aprilTagCameraBl;

    private final PhotonPoseEstimator photonEstimatorBr;
    private final PhotonPoseEstimator photonEstimatorBl;

    private double lastTimeStampFront = 0;
    private double lastEstTimestampBack = 0;

    private final double maxAcceptableRange = 2.75;

    public Vision() {
        aprilTagCameraBr = new PhotonCamera(Constants.Vision.camBrName);
        aprilTagCameraBl = new PhotonCamera(Constants.Vision.camBlName);

        Constants.Vision.aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        photonEstimatorBr = new PhotonPoseEstimator(
                Constants.Vision.aprilTagLayout,
                PoseStrategy.LOWEST_AMBIGUITY,
                aprilTagCameraBr,
                Constants.Vision.backrCameraLocation);

        photonEstimatorBl = new PhotonPoseEstimator(
                Constants.Vision.aprilTagLayout,
                PoseStrategy.LOWEST_AMBIGUITY,
                aprilTagCameraBl,
                Constants.Vision.backlCameraLocation);

        // 2024 field quality makes multitag impractical
        // photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /** Get the latest result from the front April Tag camera */
    public PhotonPipelineResult getLatestResultATF() {
        return aprilTagCameraBr.getLatestResult();
    }

    /** Get the latest result from the back April Tag camera */
    public PhotonPipelineResult getLatestResultATB() {
        return aprilTagCameraBl.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedPoseBr() {
        var visionEst = photonEstimatorBr.update();
        double latestTimestamp = aprilTagCameraBr.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastTimeStampFront) > 1e-5;
        if (newResult) lastTimeStampFront = latestTimestamp;
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseBl() {
        var visionEst = photonEstimatorBl.update();
        double latestTimestamp = aprilTagCameraBl.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestampBack) > 1e-5;
        if (newResult) lastEstTimestampBack = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedPoseBr()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevsBr(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = getLatestResultATF().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimatorBr.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        // if (numTags > 1)
        //    estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (avgDist > maxAcceptableRange)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevsBl(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = getLatestResultATB().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimatorBl.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        // if (numTags > 1)
        //    estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (avgDist > maxAcceptableRange)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
