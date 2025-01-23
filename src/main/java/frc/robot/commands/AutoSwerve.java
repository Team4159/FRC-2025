package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoSwerve extends Command{
    Trajectory traj;
    TrajectoryConfig config;
    CommandSwerveDrivetrain swerve;
    SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    Supplier<Pose2d> desiredPose;
    double timeOffset;
    Timer timer = new Timer();
    PIDController xController = new PIDController(5, 0.001, 0);
    PIDController yController = new PIDController(5, 0.001, 0);
    PIDController tController = new PIDController(5, 0.1, 0.1){{
        enableContinuousInput(-Math.PI, Math.PI);
    }};


    public AutoSwerve(CommandSwerveDrivetrain swerve, Supplier<Pose2d> desiredPose){
        this.swerve = swerve;
        this.desiredPose = desiredPose;
    }

    @Override
    public void execute(){
        double sx = xController.calculate(swerve.getState().Pose.getX(), desiredPose.get().getX());
        double sy = yController.calculate(swerve.getState().Pose.getY(), desiredPose.get().getY());
        double st = tController.calculate(swerve.getState().Pose.getRotation().getRadians(), desiredPose.get().getRotation().getRadians());
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(sx, sy, st), swerve.getState().Pose.getRotation());
        swerve.setControl(applyRobotSpeeds.withSpeeds(speeds));
    }
}
