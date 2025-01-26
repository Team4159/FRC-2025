package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class Swerve{
        //bumpers included
        public static final double width = 30;
        public static final double L4Offset = 3.5;
    }

    public static final class Elevator{
        public static final int elevatorMotorID = 1;
        public static final int limitSwitchPort = 0;

        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidProfileConstraints);

        //TODO ask Austin about mech
        public static final double rotationsPerMeter = 30;

        public static final double elevatorTolerance = 0.02;

        public static final double kS = 0;
        public static final double kV = 1;
        public static final double kA = 0;
        public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kV, kA);

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final TrapezoidProfile.Constraints PIDConstraints = new Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController elevatorPID = new ProfiledPIDController(kP, kI, kD, PIDConstraints);

        //TODO: find setpoints for elevator
        public static enum ElevatorState{
            STOW(Units.inchesToMeters(0)),
            INTAKE(Units.inchesToMeters(3)),
            L1(Units.inchesToMeters(6)),
            L2(Units.inchesToMeters(15)),
            L3(Units.inchesToMeters(20)),
            L4(Units.inchesToMeters(30));

            public double height;
            private ElevatorState(double height){
                this.height = height;
            }
        }
    }

    public static final class Field{
        //values are in inches
        public static final double poleDist = 1.652;
        public static final double fieldLength = 690.875;
        public static final double fieldWidth = 317;

        public static final List<Pose2d> reefBlue = List.of(
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 177.648 + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 29.685 + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3)), new Rotation2d(-Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 188.870 + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 23.289 + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3)), new Rotation2d(-Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 199.431 - (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  + 6.468), new Rotation2d()),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 199.431 - (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  - 6.468), new Rotation2d()),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 188.870 + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 23.289 + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3)), new Rotation2d(Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 177.648 + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 29.685 + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3)), new Rotation2d(Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 159.734 + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 29.685 + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3)), new Rotation2d(2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 147.721 + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 23.289 + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3)), new Rotation2d(2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 137.999 + (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  + 6.468), new Rotation2d(Math.PI)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 137.999 + (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  - 6.468), new Rotation2d(Math.PI)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 147.721 + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 23.289 + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3)), new Rotation2d(-2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 - 159.734 + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 29.685 + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3)), new Rotation2d(-2*Math.PI/3))
        );
        public static final List<Pose2d> reefRed = List.of(
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 177.648 + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 29.685 + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3)), new Rotation2d(-2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 188.870 + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 23.289 + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3)), new Rotation2d(-2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 199.431 + (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  + 6.468), new Rotation2d(Math.PI)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 199.431 + (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  - 6.468), new Rotation2d(Math.PI)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 188.870 + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 23.289 + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3)), new Rotation2d(2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 177.648 + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 29.685 + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3)), new Rotation2d(2*Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 159.734 + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 29.685 + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3)), new Rotation2d(Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 147.721 + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  - 23.289 + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3)), new Rotation2d(Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 137.999 - (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  + 6.468), new Rotation2d()),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 137.999 - (poleDist + Swerve.width/2)), Units.inchesToMeters(fieldWidth/2  - 6.468), new Rotation2d()),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 147.721 + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 23.289 + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3)), new Rotation2d(-Math.PI/3)),
            new Pose2d(Units.inchesToMeters(fieldLength/2 + 159.734 + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3)), Units.inchesToMeters(fieldWidth/2  + 29.685 + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3)), new Rotation2d(-Math.PI/3))
        );
        public static final Map<DriverStation.Alliance, List<Pose2d>> reef = Map.of(
            Alliance.Blue, reefBlue,
            Alliance.Red, reefRed
        );
    }
}
