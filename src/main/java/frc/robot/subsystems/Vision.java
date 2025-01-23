package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase{
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    CommandSwerveDrivetrain drivetrain;
    boolean doRejectUpdate;
    ShuffleboardTab visionTab;
    Field2d visionField;

    public Vision(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        visionTab = Shuffleboard.getTab("Vision");
        visionField = new Field2d();
    }

    @Override
    public void periodic(){
        LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 720*Math.PI/180) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, Timer.getFPGATimestamp() - mt2.timestampSeconds);
            visionField.setRobotPose(mt2.pose);
            Shuffleboard.getTab("Vision").add("Vision Pose", visionField);
        }
    }
}
