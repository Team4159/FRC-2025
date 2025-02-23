package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoSwerve extends Command{
    protected CommandSwerveDrivetrain swerve;
    private SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private Supplier<Pose2d> desiredPoseSupplier;
    protected Pose2d desiredPose;
    protected Transform2d transform;
    public AutoSwerve(CommandSwerveDrivetrain swerve){
        this.swerve = swerve;
    }

    public AutoSwerve(CommandSwerveDrivetrain swerve, Pose2d desiredPose){
        this.swerve = swerve;
        this.desiredPose = desiredPose;
    }

    public AutoSwerve(CommandSwerveDrivetrain swerve, Supplier<Pose2d> desiredPose){
        this.swerve = swerve;
        this.desiredPoseSupplier = desiredPose;
    }

    @Override
    public void initialize(){
        Constants.Swerve.translationController.reset(0);
        Constants.Swerve.rotationController.reset(swerve.getState().Pose.getRotation().getRadians());
        transform = desiredPose.minus(swerve.getState().Pose);
    }

    @Override
    public void execute(){
        if(desiredPoseSupplier != null)
            desiredPose = desiredPoseSupplier.get();
        Pose2d robotPose = swerve.getState().Pose;
        double angle = desiredPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        double distance = desiredPose.getTranslation().getDistance(robotPose.getTranslation());
        double translationSpeed = Math.abs(Constants.Swerve.translationController.calculate(distance, 0));
        double sx = translationSpeed * Math.signum(desiredPose.getX() - robotPose.getX()) * Math.abs(Math.cos(angle));
        double sy = translationSpeed * Math.signum(desiredPose.getY() - robotPose.getY()) * Math.abs(Math.sin(angle));
        double st = Constants.Swerve.rotationController.calculate(robotPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(sx, sy, st), robotPose.getRotation());
        swerve.setControl(applyRobotSpeeds.withSpeeds(speeds));
    }
}
