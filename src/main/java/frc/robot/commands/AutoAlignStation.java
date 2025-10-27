package frc.robot.commands;

import javax.security.auth.RefreshFailedException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;

public class AutoAlignStation extends AutoSwerve{
    private boolean left;
    private boolean tooFar;
    private LED led;
    private Field2d f2d = new Field2d();

    /**
     * @param swerve CommandSwerveDrivetrain subsystem for the swerve drivetrain
     * @param led LED subsystem(Status Lights: Red Blink = too far to align, Yellow Blink = aligning, Green Blink = aligned)
     * @param left If true the robot will align to the robot relative left closest reef pole, otherwise it will align to the robot relative closest right pole
     */
    public AutoAlignStation(CommandSwerveDrivetrain swerve, Elevator elevator, LED led){
        super(swerve, elevator);
        this.led = led;
    }

    @Override
    public void initialize(){
        var reefPoses = Constants.Field.stations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        desiredPose = swerve.getState().Pose.nearest(reefPoses);
        f2d.setRobotPose(desiredPose);
        SmartDashboard.putData("autostation f2d", f2d);
        if(desiredPose.minus(swerve.getState().Pose).getTranslation().getNorm() < Constants.Swerve.maxReefAutoAlignDistatnce){
            tooFar = false;
            super.initialize();
        }
        else{
            tooFar = true;
        }
    }

    @Override
    public void execute(){
        if(tooFar) return;
        super.execute();
    }

    @Override
    public boolean isFinished(){
        if(tooFar) return true;
        Pose2d swervePose = swerve.getState().Pose;
        if(MathUtil.isNear(desiredPose.getX(), swervePose.getX(), Constants.Swerve.translationTolerance)
        && MathUtil.isNear(desiredPose.getY(), swervePose.getY(), Constants.Swerve.translationTolerance)
        && MathUtil.isNear(desiredPose.getRotation().getRadians(), swervePose.getRotation().getRadians(), Constants.Swerve.rotationTolerance)){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(tooFar){
            led.blink(Color.kRed, 0.25);
        }
        else{
            led.blink(Color.kGreen, 0.25);
        }
    }
}

