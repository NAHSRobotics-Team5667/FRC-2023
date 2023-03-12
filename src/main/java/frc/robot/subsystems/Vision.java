package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

public class Vision {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose> currentFieldPose;
    
    public Vision() {
        camera = new PhotonCamera("USB Camera 0");
        poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.robotToCamera);
        currentFieldPose = Optional.empty();
    }
    
    /**
    * @return
    */
    
    public Optional<EstimatedRobotPose> getCurrentPoseEstimate() {
        return currentFieldPose;
    }
    
    
    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }
    
    public void run() {
        Optional<EstimatedRobotPose> poseResult = poseEstimator.update();
        if (poseResult.isPresent()) {
            currentFieldPose = poseResult;
        } else {
            currentFieldPose = Optional.empty();
        }
    }
}