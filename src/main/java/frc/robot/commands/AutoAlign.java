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
    // private Trajectory traj;
    // private boolean trajMode;
    private boolean tooFar;
    // private Timer timer;
    private LED led;

    // public AutoAlign(CommandSwerveDrivetrain swerve){
    //     this(swerve, false, false);
    // }

    // public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4){
    //     this(swerve, L4, false);
    // }

    public AutoAlign(CommandSwerveDrivetrain swerve, LED led, boolean left){
        super(swerve);
        this.left = left;
        this.led = led;
        // timer = new Timer();
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
            //desiredPose = swerve.getDesieredAutoAlignPose(L4, secondClosestPose);
            var reefPoses = Constants.Field.reef.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
            var closestSide = swerve.getState().Pose.nearest(reefPoses);
            var angle = closestSide.getRotation().getRadians() - Math.PI/2;
            if(left){
                desiredPose = new Pose2d(closestSide.getX() - Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() - Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            }
            else{
                desiredPose = new Pose2d(closestSide.getX() + Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() + Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            }
            // traj = TrajectoryGenerator.generateTrajectory(swerve.getState().Pose, new ArrayList<Translation2d>(), desiredPose, new TrajectoryConfig(1, 1));
            // if(traj.equals(new Trajectory(List.of(new Trajectory.State())))){
            //     trajMode = false;
            // }
            // else{
            //     trajMode = true;
            // }
            // timer.restart();
            led.blink(Color.kYellow);
        }
        else{
            tooFar = true;
        }
        super.initialize();
    }

    @Override
    public void execute(){
        if(tooFar) return;
        // if(trajMode){
        //     System.out.println("traj");
        //     var state = traj.sample(timer.get());
        //     // swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        //     //     controller.calculate(kinesthetics.getPose(), state, state.poseMeters.getRotation())
        //     // ));
        //     ChassisSpeeds speeds = Constants.Swerve.holonomicController.calculate(swerve.getState().Pose, state, state.poseMeters.getRotation());
        //     speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, swerve.getState().Pose.getRotation());
        //     swerve.setControl(applyRobotSpeeds.withSpeeds(speeds));
        // }
        // else{
            //System.out.println("pid");
            super.execute();
        // }
        //System.out.println(swerve.getDesieredAutoAlignPose(L4, secondClosestPose).getX());
    }

    @Override
    public boolean isFinished(){
        if(tooFar) return true;
        Pose2d swervePose = swerve.getState().Pose;
        if(MathUtil.isNear(desiredPose.getX(), swervePose.getX(), 0.01)
        && MathUtil.isNear(desiredPose.getY(), swervePose.getY(), 0.01)
        && MathUtil.isNear(desiredPose.getRotation().getRadians(), swervePose.getRotation().getRadians(), Math.PI/64)){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(tooFar){
            led.blink(Color.kRed);
        }
        else{
            led.blink(Color.kGreen);
        }
    }
}

// public class AutoAlign extends TrajSwerve{
//     boolean L4, secondClosestPose;

//     public AutoAlign(CommandSwerveDrivetrain swerve){
//         this(swerve, false, false);
//     }

//     public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4){
//         this(swerve, L4, false);
//     }

//     public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4, boolean secondClosestPose){
//         super(swerve);
//         this.L4 = L4;
//         this.secondClosestPose = secondClosestPose;
//     }

//     @Override
//     public void initialize(){
//         setDesiredPose(swerve.getDesieredAutoAlignPose(L4, secondClosestPose));
//         super.initialize();
//     }

//     @Override
//     public void execute(){
//         super.execute();
//         //System.out.println(swerve.getDesieredAutoAlignPose(L4, secondClosestPose).getX());
//     }
// }

