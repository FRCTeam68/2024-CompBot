package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

    /**
     * The Vision class is a class which sees
     * It sees all and knows all.
     * It can see the stars upon the heavens
     * See even the little bugs on the ground
     * It can see into your soul
     * And into your darkest sins
     * 
     * You cannot hide from this class here
     * All will not be well
     * 
     * The Vision here is what our Robot knows
     * What this sees, it sees
     * We process it all here and grind it up
     * Processed food yuck!
     * <h1>Head for your cats</h1>
     * 
     * @hidden Do not let it eat after midnight
     * @author ajmoritz2
     */
public class Vision {


    public enum Camera {
        FRONT,
        BACK
    }

    private InterpolatingTreeMap<Double, Double> speakerMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    private PhotonCamera cameraInUse;
    private PhotonPipelineResult resultsInUse;
    private Transform3d currentCamTrans;
    private AprilTagFieldLayout layout;

    private PhotonCamera camF;
    private PhotonCamera camB;

    // {Distance, Shooter angle} -> Linear InterORIation
    private double[][] distanceAngles = {{1.74, 0}, {2.69, 12}, {3.69, 28}, {4.65, 32}, {5.65, 38}, {6.70, 40}};

    public Vision(){
        camF = new PhotonCamera(Constants.Vision.camLName);
        camB = new PhotonCamera(Constants.Vision.camRName);
        layout =  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        for (double[] vals : distanceAngles) {
            // ULTRAKILL
            speakerMap.put(vals[0], vals[1]);
        }
        /* Keeping for might later use
        fPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.frontCameraLocation);
        bPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.backCameraLocation);
        */
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
            return TunerConstants.steerPID.calculate(resultsInUse.getBestTarget().getYaw(), 0);
        }

        return 0;
    }

    // Probably the most useful one
    /**
     * @apiNote Probably best to refresh the target every cycle for ACCURACY! (Sein would do it)
     * @param target
     * @return
     */
    public double aimWithYawAtTarget(PhotonTrackedTarget target) {
        // Ori, what are you doing!?
        return TunerConstants.steerPID.calculate(Math.toRadians(target.getYaw()), Math.toRadians(Robot.m_robotContainer.m_DriveSubSystem.getPigeon2().getYaw().getValue()));
    }

    //Slightly less useful one
    /** This one drives forward only. I could make one for swerve but it might be better for it to just go forward.
     * 
     * @hidden May you have luck through the Blind Forest
     * @param target
     * @param targetHeight 
     * @param targetPitch
     * @param dMeters Distance you WANT to be from the target
     * @return the value to shove through the y drive, PID included
     */
    public double driveDistFromTarget(PhotonTrackedTarget target, double targetHeight, double targetPitch, double dMeters) {
        
        return TunerConstants.drivePID.calculate(PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Vision.frontCameraLocation.getY(), targetHeight, Constants.Vision.frontCameraLocation.getRotation().getY()
            , targetPitch), dMeters);
    }

    public double shooterSpeedFromDist(PhotonTrackedTarget target){
        System.out.println("Distance from target X: %s | Distance from target Y: %s".formatted(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY()));
        return speakerMap.get(Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(),2) + Math.pow(target.getBestCameraToTarget().getY(), 2)));
    }

    public PhotonTrackedTarget getFiscalIDTarget(int id, List<PhotonTrackedTarget> visibleTargets) {
        if (visibleTargets == null){
            return null;
        }
        if (visibleTargets.isEmpty())
            return null;
        for (PhotonTrackedTarget targ : visibleTargets) {
            if (targ.getFiducialId() == id)
                return targ;
        }

        return null;
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
