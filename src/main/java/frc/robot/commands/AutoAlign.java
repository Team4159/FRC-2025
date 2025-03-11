package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends AutoSwerve{
    boolean L4, secondClosestPose;

    public AutoAlign(CommandSwerveDrivetrain swerve){
        this(swerve, false, false);
    }

    public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4){
        this(swerve, L4, false);
    }

    public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4, boolean secondClosestPose){
        super(swerve);
        this.L4 = L4;
        this.secondClosestPose = secondClosestPose;
    }

    @Override
    public void initialize(){
        desiredPose = swerve.getDesieredAutoAlignPose(L4, secondClosestPose);
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
        //System.out.println(swerve.getDesieredAutoAlignPose(L4, secondClosestPose).getX());
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

