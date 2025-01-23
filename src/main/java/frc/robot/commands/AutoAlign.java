package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends SequentialCommandGroup{
    Supplier<Pose2d> target;

    public AutoAlign(CommandSwerveDrivetrain swerve){
        this(swerve, false);
    }

    public AutoAlign(CommandSwerveDrivetrain swerve, boolean L4){
        target = () -> swerve.getClosestReef(L4);
        addCommands(new AutoSwerve(swerve, target));
    }
}
