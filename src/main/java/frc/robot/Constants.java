package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class Swerve{
        //bumpers included
        public static final double width = 34;
        public static final double L4Offset = 3.5;
    }

    public static final class Elevator{
        public static final int elevatorMotorID = 1;
        public static final int limitSwitchPort = 0;

        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidProfileConstraints);

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kV, kA);

        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final PIDController elevatorPID = new PIDController(kP, kI, kD);
    }

    public static final class AlgaeIntake {
        public static final int rollerID = 3, pivotID = 2;

        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final PIDController pidController = new PIDController(kP, kI, kD){{
            enableContinuousInput(0, 2*Math.PI);
        }};

        public static enum AlgaeIntakeState {
            STOW(1.85, 0),
            INTAKE(0.7, 1),
            OUTTAKE(0.7, -1);

            public final double speed;
            public final double angle;
            private AlgaeIntakeState(double a, double s) {
                speed = s;
                angle = a;
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
