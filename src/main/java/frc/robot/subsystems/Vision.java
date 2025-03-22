package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase{
    private CommandSwerveDrivetrain drivetrain;
    private Field2d visionField;

    public Vision(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        visionField = new Field2d();
        Shuffleboard.getTab("Vision").add("visionPose", visionField);
    }

    @Override
    public void periodic() {
        visionField.setRobotPose(updateOdometry());
    }
    
    /** Updates the field relative position of the robot. */
    public Pose2d updateOdometry() {
        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            drivetrain.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
            return mt1.pose;
        }
        }
        else if (useMegaTag2 == true)
        {
        LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
            return mt2.pose;
        }
        }
        return null;
    }
}
