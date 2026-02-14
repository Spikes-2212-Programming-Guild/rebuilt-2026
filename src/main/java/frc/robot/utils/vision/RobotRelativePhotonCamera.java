package frc.robot.utils.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class RobotRelativePhotonCamera implements RobotRelativeVisionSource {

    private final PhotonCamera camera;
    private PhotonPipelineResult lastResult;

    public RobotRelativePhotonCamera(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
        this.lastResult = new PhotonPipelineResult();
    }

    private PhotonPipelineResult getResult() {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

        if (!unreadResults.isEmpty()) {
            lastResult = unreadResults.get(unreadResults.size() - 1);
        }

        return lastResult;
    }

    @Override
    public boolean hasValidTarget() {
        return getResult().hasTargets();
    }

    @Override
    public double getHorizontalOffset() {
        PhotonPipelineResult result = getResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    @Override
    public double getTargetArea() {
        PhotonPipelineResult result = getResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getArea();
        }
        return 0.0;
    }

    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }
}
