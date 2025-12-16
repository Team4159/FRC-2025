package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase{
    private CommandSwerveDrivetrain drivetrain;
    private AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private PhotonCamera leftCam, rightCam, algaeCam;
    private PhotonPoseEstimator leftEstimator, rightEstimator;
    private double timeOffset;

    public PhotonVision(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        //apriltag
        leftCam = new PhotonCamera("leftCam");
        rightCam = new PhotonCamera("rightCam");
        leftEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.leftRobotToCameraTransform);
        rightEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.rightRobotToCameraTransform);
        timeOffset = Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp();
        //object detection
        algaeCam = new PhotonCamera("algaeDetection");
        // SmartDashboard.putNumber("time offset", timeOffset);
    }

    @Override
    public void periodic(){
        //left camera
        Optional<EstimatedRobotPose> leftEstimate = Optional.empty();
        //loops through all unread camera results
        for(PhotonPipelineResult leftCamResult : leftCam.getAllUnreadResults()){
            //get pose estimate
            leftEstimate = leftEstimator.update(leftCamResult);
            //check if estimate exists
            if(leftEstimate.isPresent()){
                //set standard deviation
                drivetrain.setVisionMeasurementStdDevs(calculateEstimationStdDevs(leftEstimate, leftCamResult.targets));
                //send the pose estimate to the pose estimator
                drivetrain.addVisionMeasurement(leftEstimate.get().estimatedPose.toPose2d(), leftEstimate.get().timestampSeconds + timeOffset);
            }
        }

        //right camera
        Optional<EstimatedRobotPose> rightEstimate = Optional.empty();
        //loops through all unread camera results
        for(PhotonPipelineResult rightCamResult : rightCam.getAllUnreadResults()){
            //get pose estimate
            rightEstimate = rightEstimator.update(rightCamResult);
            //check if estimate exists
            if(rightEstimate.isPresent()){
                //set standard deviation
                drivetrain.setVisionMeasurementStdDevs(calculateEstimationStdDevs(rightEstimate, rightCamResult.targets));
                //send the pose estimate to the pose estimator
                drivetrain.addVisionMeasurement(rightEstimate.get().estimatedPose.toPose2d(), rightEstimate.get().timestampSeconds + timeOffset);
            }
        }

        for(PhotonPipelineResult algaeCamResult : algaeCam.getAllUnreadResults()){
            var algae = algaeCamResult.getBestTarget();
            if(algae == null) return;
            var confidence = algae.getDetectedObjectConfidence();
            var pose = algae.getYaw();
        }
    }

    public Double getAlgaeYaw(){
        PhotonPipelineResult result = algaeCam.getLatestResult();
        var algae = result.getBestTarget();
        if(algae == null || algae.getDetectedObjectConfidence() < 0.2) return null;
        return algae.getYaw();
    }

    //default photonvision stddev calculator
    // private Vector<N3> calculateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
    //         Vector<N3> stddevs;
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         stddevs = kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
    //             numTags++;
    //             avgDist +=
    //                     tagPose
    //                             .get()
    //                             .toPose2d()
    //                             .getTranslation()
    //                             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             stddevs = kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             stddevs = estStdDevs;
    //         }
    //     }
    //     return stddevs;
    // }

    private Vector<N3> calculateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        //range should be form 0(no tag) to 1(full coverage) (limelight standard)
        //photonvision area is scaled from 0-100 so need to convert
        double area = 0;
        //calculate area of all targets
        if(estimatedPose.isPresent()){
            for(var tag : targets){
                //convert the scaling
                area += tag.area/100; 
            }
            System.out.println("stddev position: " + (1 - area * 0.3));
            //TODO: tune, currently this is just the limelight one(why are sds on limelight negative lol)
            return VecBuilder.fill(1 - area * 0.3, 1 - area * 0.3, 1-area * 0.1);
        }
        //if the estimated pose does not exist just return extremely high stddevs
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
}
