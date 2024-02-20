package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;

public class Vision {

    public int camLNum = 0, camRNum = 1;

    private PhotonCamera camL;
    private PhotonCamera camR;

    public Vision(){
        camL = new PhotonCamera(Constants.Vision.camLName);
        camR = new PhotonCamera(Constants.Vision.camRName);
    }

    public PhotonPipelineResult getLeftResult(){
        return camL.getLatestResult();
    }

    public PhotonPipelineResult getRightResult(){
        return camR.getLatestResult();
    }

    public void photoBoth(){
        takePhotoLeft();
        takePhotoRight();
    }

    public void takePhotoLeft() {
        camL.takeInputSnapshot();
        camL.takeOutputSnapshot();
    }

    public void takePhotoRight() {
        camR.takeInputSnapshot();
        camR.takeOutputSnapshot();
    }

    public PhotonCamera getCamera(int cam) {
        return (cam == camLNum ? camL : camR); // Getter!
    }
}
