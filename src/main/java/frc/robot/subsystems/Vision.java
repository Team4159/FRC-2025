package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private Field2d visionField;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionField = new Field2d();
        Shuffleboard.getTab("Vision").add("visionPose", visionField);
    }

    @Override
    public void periodic() {
        var est = bestEstimate();
        if (est == null) {
            visionField.setRobotPose(new Pose2d());
            return;
        }
        if (est.isMegaTag2) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(
                est.pose,
                est.timestampSeconds);
        } else {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            drivetrain.addVisionMeasurement(
                    est.pose,
                    est.timestampSeconds);
        }
        visionField.setRobotPose(est.pose);
    }

    /** Updates the field relative position of the robot. */
    public LimelightHelpers.PoseEstimate bestEstimate() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (
            mt1.tagCount == 0 ||
            (mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && (
                mt1.rawFiducials[0].ambiguity > .7 ||
                mt1.rawFiducials[0].distToCamera > 3
            ))
        ) {
            mt1 = null;
        }
        
        LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (
            Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > Math.PI * 2 ||
            mt2.tagCount == 0
        ) {
            mt2 = null;
        }

        if (mt1 == null && mt2 == null) return null;
        if (mt1 == null && mt2 != null) return mt2;
        if (mt2 == null && mt1 != null) return mt1;
        
        if (mt2.tagCount > 1) return mt2;
        if (mt1.rawFiducials[0].ta > 0.7) return mt1;

        return null;
    }
}
