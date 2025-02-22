package frc.robot.commands;

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
}
