package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
     * Quiet please, it is seeing.
     * <hr>
     * <code>
     * Full fathom five thy father lies; <hr>
     * Of his bones are coral made; <hr>
     * Those are pearls that were his eyes; <br>
     * Nothing of him that doth fade, <br>
     * But doth suffer a sea-change <br>
     * Into something rich and strange. <br>
     * Sea-nymphs hourly ring his knell: <br>
     * Ding-dong. <br>
     * Hark! now I hear them -- Ding-dong, bell. </code>
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

    public PhotonPoseEstimator blPoseEst;
    public PhotonPoseEstimator brPoseEst;
    public PhotonPipelineResult brResult;

    public PhotonPipelineResult blResult;


    public EstimatedRobotPose estPose = null;
    public Pose2d pastPose = new Pose2d(0,0,new Rotation2d(0,0));

    private PhotonCamera camBr;
    private PhotonCamera camBl;

    private int[] stageTags;

    // {Distance, Shooter angle} -> Linear InterORIation
    private double[][] distanceAngles = {{1.74, 0}, {2.69, 12}, {3.69, 28}, {4.65, 32}, {5.65, 38}, {6.70, 40}};

    public boolean backOdoUpdated = false;

    public Vision(){
        camBr = new PhotonCamera(Constants.Vision.camBrName);
        camBl = new PhotonCamera(Constants.Vision.camBlName);
        layout =  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        for (double[] vals : distanceAngles) {
            // ULTRAKILL
            speakerMap.put(vals[0], vals[1]);
        }
        /* Keeping for might later use */
        // fPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.frontCameraLocation);
        
        // Make sure we enable multi-targets on the Camera. 
        blPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camBl, Constants.Vision.backlCameraLocation);
        blPoseEst.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        brPoseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camBr, Constants.Vision.backrCameraLocation);
        brPoseEst.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // stageTags = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Constants.BLUE_TAGS.stage : Constants.RED_TAGS.stage;
        
        stageTags = Constants.BLUE_TAGS.stage;
        cameraInUse = camBr;
        currentCamTrans = Constants.Vision.frontCameraLocation;

    }


    // Pose estimationg stuff
    public Pose2d estimatePoseBack(){
        if (estPose == null)
            return null;        
        return estPose.estimatedPose.toPose2d();

    }

    public EstimatedRobotPose estimatedRobotPose() {
        return estPose;
    }
    // Getting more specific
   
    public double distanceToStage() {
        if (!resultsInUse.hasTargets()) 
            return 0;
        if (resultsInUse.getTargets().size() != 1)
            return 0;
        PhotonTrackedTarget target = resultsInUse.getBestTarget();
        for (int tagID : stageTags) {
            
            if (target.getFiducialId() == tagID) {
                return target.getBestCameraToTarget().getX();
            } 
        }
        
        return 0;
    }

    @Deprecated
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

    public int getBrTagCount() {
        if (brResult == null) {
            return 0;
        }
        return brResult.getTargets().size();
    }

    public int getBlTagCount() {
        if (blResult == null) {
            return 0;
        }
        return blResult.getTargets().size();
    }

    /**
     * <strong>April tag</strong> target!
     * @param target
     * @return radians angle between target? TEST
     */
    public double getYawToTarget(PhotonTrackedTarget target) {
        Pose2d tagPose = layout.getTagPose(target.getFiducialId()).get().toPose2d();
        Pose2d robotPose = estimatePoseBack();
        
        double dx = tagPose.getX() - robotPose.getX();
        double dy = tagPose.getY() - tagPose.getY();

        double hyp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        double angle = Math.acos(dx/hyp);

        return angle;
    }

    // Probably the most useful one
    /**
     * @apiNote Probably best to refresh the target every cycle for ACCURACY! (Sein would do it)
     * @param target
     * @return
     */
    public double aimWithYawAtTarget(PhotonTrackedTarget target) {
        // Ori, what are you doing!? So verbose! 
        return TunerConstants.steerPID.calculate(getYawToTarget(target), Robot.m_robotContainer.m_DriveSubSystem.getOdometry().getEstimatedPosition().getRotation().getRadians()); // MANKIND IS DEAD
        //TODO: I'll probably have to switch those values around and maybe add an offset of sorts because I dont really know how it works at all.
   
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

    // Target stuff /////////////////////////////////////////////////

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

    public List<PhotonTrackedTarget> getCurrentTargets(){ // BLOOD IS FUEL
        if (!resultsInUse.hasTargets())
            return null;
        return resultsInUse.getTargets();
    }

    public PhotonTrackedTarget getCurrentBestTarget(){
        if (!resultsInUse.hasTargets())
            return null;
        return resultsInUse.getBestTarget();
    }



    // Just a bunch of Camera stuff because it needs to be there

    /**
     * Call once every loop?
     */
    public void updateCurrentCam(){
        PhotonPipelineResult backResult = camBl.getLatestResult();
        brResult = camBr.getLatestResult();
        blResult = backResult;
    }

    public void updatebrCam(){
        PhotonPipelineResult backResult = camBr.getLatestResult();
        brResult = backResult;

        Optional<EstimatedRobotPose> isEst = brPoseEst.update(backResult);

        if (isEst.isPresent()){
            if (estPose != null){
                pastPose = estPose.estimatedPose.toPose2d();
            } else {
                pastPose = new Pose2d();
            }
            estPose = isEst.get();
            
        }
    }
    
    public void updateblCam(){
        PhotonPipelineResult backResult = camBl.getLatestResult();
        blResult = backResult;
        Optional<EstimatedRobotPose> isEst = blPoseEst.update(backResult);

        if (isEst.isPresent()){
            if (estPose != null){
                pastPose = estPose.estimatedPose.toPose2d();
            } else {
                pastPose = new Pose2d();
            }
            estPose = isEst.get();
            
        }
    }
    public PhotonPipelineResult getFrontResult(){
        return camBr.getLatestResult();
    }

    public PhotonPipelineResult getBackResult(){
        return camBl.getLatestResult();
    }

    public void photoBoth(){ // HELL IS FULL
        takePhotoFront();
        takePhotoBack();
    }

    public void takePhotoFront() {
        camBr.takeInputSnapshot();
        camBr.takeOutputSnapshot();
    }

    public void takePhotoBack() {
        camBl.takeInputSnapshot();
        camBl.takeOutputSnapshot();
    }

    public void setUsedCamera(Camera cam) {
        if (cam == Camera.FRONT) {
            currentCamTrans = Constants.Vision.frontCameraLocation;
            cameraInUse = camBr;
        } else if (cam == Camera.BACK) {
            currentCamTrans = Constants.Vision.backlCameraLocation;
            cameraInUse = camBl;
        }
    }

    /**
     * Swaps between back and front camera
     */
    public void swapCamera() {
        if (cameraInUse.equals(camBl)){
            setUsedCamera(Camera.FRONT);
        } else {
            setUsedCamera(Camera.BACK);
        }
    }

    public PhotonCamera getCamera(Camera cam) {
        return (cam == Camera.FRONT ? camBr : camBl); // Getter!
    }

    public boolean goodResult(PhotonPipelineResult result) {
        return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < Constants.Vision.APRILTAG_AMBIGUITY_THRESHOLD
                && layout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                        .getDistance(Robot.m_robotContainer.m_DriveSubSystem.lastEstimate.getTranslation()) < Constants.Vision.MAX_DISTANCE;
    }

    public Pose2d getAverageEstimate(PhotonPipelineResult[] results,
            PhotonPoseEstimator[] photonEstimator) {
        double x = 0;
        double y = 0;
        double rot = 0;
        int count = 0;
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        for (int i = 0; i < results.length; i++) {
            if (results[i] == null)
                continue;
            PhotonPipelineResult result = results[i];
            if (result.hasTargets()) {
                var est = photonEstimator[i].update();
                if (est.isPresent() && goodResult(result)) {
                    x += est.get().estimatedPose.getTranslation().getX();
                    y += est.get().estimatedPose.getTranslation().getY();
                    rot += est.get().estimatedPose.getRotation().getAngle();
                    poses.add(est.get().estimatedPose.toPose2d());
                    count++;
                }
            }
        }
        if (count == 0)
            return new Pose2d();
        Pose2d[] poseArray = new Pose2d[poses.size()];
        for (int i = 0; i < poses.size(); i++) {
            poseArray[i] = poses.get(i);
        }
        return new Pose2d(x / count, y / count, new Rotation2d(rot / count));
    }

    public Matrix<N3, N1> getEstimationStdDevsBl(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = blResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = blPoseEst.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevsBr(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = brResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = brPoseEst.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}