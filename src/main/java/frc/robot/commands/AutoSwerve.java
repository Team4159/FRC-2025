package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoSwerve extends Command{
    protected CommandSwerveDrivetrain swerve;
    protected SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /** Field Centric closed loop request for teleop driving */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity 0.75 old value
    protected SwerveRequest.FieldCentric m_fieldCentricClosedLoopRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.Velocity); // use closed loop control for drive motors
    protected Pose2d desiredPose;

    /** only used for child classes that set desiredPose manually */
    protected AutoSwerve(CommandSwerveDrivetrain swerve){
        this.swerve = swerve;
    }

    /** @param desiredPose the desired final pose of the robot */
    public AutoSwerve(CommandSwerveDrivetrain swerve, Pose2d desiredPose){
        this.swerve = swerve;
        this.desiredPose = desiredPose;
    }

    @Override
    public void initialize(){
        Constants.Swerve.translationController.reset(0);
        Constants.Swerve.rotationController.reset(swerve.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute(){
        Pose2d robotPose = swerve.getState().Pose;
        //angle between desired and robot poses
        double angle = desiredPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        //distance from desiredPose to robotPose(hypotenuse of x and y, always positive)
        double distance = desiredPose.getTranslation().getDistance(robotPose.getTranslation());
        //calculated speed from PID controller
        double translationSpeed = Math.abs(Constants.Swerve.translationController.calculate(distance, 0));
        //calculate chassis speeds
        double sx = translationSpeed * Math.signum(desiredPose.getX() - robotPose.getX()) * Math.abs(Math.cos(angle));
        double sy = translationSpeed * Math.signum(desiredPose.getY() - robotPose.getY()) * Math.abs(Math.sin(angle));
        double st = Constants.Swerve.rotationController.calculate(robotPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());
        //ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(sx, sy, st), robotPose.getRotation());
        //apply chassis speeds to the robot
        swerve.setControl(m_fieldCentricClosedLoopRequest.withVelocityX(-sx).withVelocityY(-sy).withRotationalRate(st));
    }

     @Override
    public boolean isFinished(){
        Pose2d swervePose = swerve.getState().Pose;
        if(MathUtil.isNear(desiredPose.getX(), swervePose.getX(), 0.01)
        && MathUtil.isNear(desiredPose.getY(), swervePose.getY(), 0.01)
        && MathUtil.isNear(desiredPose.getRotation().getRadians(), swervePose.getRotation().getRadians(), Math.PI/64)){
            return true;
        }
        return false;
    }
}
