package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Vision {

    public enum Camera {
        FRONT,
        BACK
    }

    private PhotonCamera cameraInUse;
    private PhotonPipelineResult resultsInUse;
    private Transform3d currentCamTrans;
    private AprilTagFieldLayout layout;

    private PhotonPoseEstimator fPoseEst, bPoseEst;

    private PhotonCamera camF;
    private PhotonCamera camB;

    public Vision(){
        camF = new PhotonCamera(Constants.Vision.camLName);
        camB = new PhotonCamera(Constants.Vision.camRName);
        layout =  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        fPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.frontCameraLocation);
        bPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.backCameraLocation);

        cameraInUse = camF;
        currentCamTrans = Constants.Vision.frontCameraLocation;
    }


    public Pose3d getRobotLocation(PhotonTrackedTarget target){
        return PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(), layout.getTagPose(target.getFiducialId()).get(), currentCamTrans);
    }

    // Aims at the best April Tag
    public double aimWithYaw() {
        if (resultsInUse.hasTargets()) {
            return TunerConstants.steerPid.calculate(resultsInUse.getBestTarget().getYaw(), 0);
        }

        return 0;
    }

    public List<PhotonTrackedTarget> getCurrentTargets(){
        if (!resultsInUse.hasTargets())
            return null;
        return resultsInUse.getTargets();
    }

    public PhotonTrackedTarget getCurrentBestTarget(){
        if (!resultsInUse.hasTargets())
            return null;
        return resultsInUse.getBestTarget();
    }
    /**
     * Call once every loop?
     */
    public void updateCurrentCam(){
        resultsInUse = cameraInUse.getLatestResult();
    }

    public PhotonPipelineResult getFrontResult(){
        return camF.getLatestResult();
    }

    public PhotonPipelineResult getBackResult(){
        return camB.getLatestResult();
    }

    public void photoBoth(){
        takePhotoFront();
        takePhotoBack();
    }

    public void takePhotoFront() {
        camF.takeInputSnapshot();
        camF.takeOutputSnapshot();
    }

    public void takePhotoBack() {
        camB.takeInputSnapshot();
        camB.takeOutputSnapshot();
    }

    public void setUsedCamera(Camera cam) {
        if (cam == Camera.FRONT) {
            currentCamTrans = Constants.Vision.frontCameraLocation;
            cameraInUse = camF;
        } else if (cam == Camera.BACK) {
            currentCamTrans = Constants.Vision.backCameraLocation;
            cameraInUse = camB;
        }
    }

    /**
     * Swaps between back and front camera
     */
    public void swapCamera() {
        if (cameraInUse.equals(camB)){
            setUsedCamera(Camera.FRONT);
        } else {
            setUsedCamera(Camera.BACK);
        }
    }

    public PhotonCamera getCamera(Camera cam) {
        return (cam == Camera.FRONT ? camF : camB); // Getter!
    }
}
