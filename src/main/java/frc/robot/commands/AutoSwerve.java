package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoSwerve extends Command{
    private CommandSwerveDrivetrain swerve;
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private Supplier<Pose2d> desiredPose;
    private PIDController xController = new PIDController(5, 0.04, 0);
    private PIDController yController = new PIDController(5, 0.04, 0);
    private PIDController tController = new PIDController(5, 0.1, 0){{
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

    @Override
    public boolean isFinished(){
        boolean finished =  MathUtil.isNear(desiredPose.get().getX(), swerve.getState().Pose.getX(), Constants.Swerve.autoSwerveToleranceXY)
        && MathUtil.isNear(desiredPose.get().getY(), swerve.getState().Pose.getY(), Constants.Swerve.autoSwerveToleranceXY)
        && MathUtil.isNear(desiredPose.get().getRotation().getRadians(), swerve.getState().Pose.getRotation().getRadians(), Constants.Swerve.autoSwerveToleranceTheta);
        //return finished;
        return false;
    }

    public void end(boolean interrupted){
        swerve.setControl(brake);
    }
}
