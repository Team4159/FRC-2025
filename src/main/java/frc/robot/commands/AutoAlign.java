package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;

public class AutoAlign extends AutoSwerve{
    private boolean left;
    private boolean tooFar;
    private LED led;

    /**
     * @param swerve CommandSwerveDrivetrain subsystem for the swerve drivetrain
     * @param led LED subsystem(Status Lights: Red Blink = too far to align, Yellow Blink = aligning, Green Blink = aligned)
     * @param left If true the robot will align to the robot relative left closest reef pole, otherwise it will align to the robot relative closest right pole
     */
    public AutoAlign(CommandSwerveDrivetrain swerve, LED led, boolean left){
        super(swerve);
        this.left = left;
        this.led = led;
    }

    @Override
    public void initialize(){
        Translation2d reefTranslation;
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 - Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        else
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 + Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        //check if close enough
        if(swerve.getState().Pose.getTranslation().getDistance(reefTranslation) < Constants.Swerve.maxReefAutoAlignDistatnce){
            var reefPoses = Constants.Field.reef.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
            var closestSide = swerve.getState().Pose.nearest(reefPoses);
            var angle = closestSide.getRotation().getRadians() - Math.PI/2;
            if(left){
                desiredPose = new Pose2d(closestSide.getX() - Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() - Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            }
            else{
                desiredPose = new Pose2d(closestSide.getX() + Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() + Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            }
            tooFar = false;
            led.blink(Color.kYellow, 0.25);
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

