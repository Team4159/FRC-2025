package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ArmFeedforward;
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

        //TODO ask Austin about mech
        public static final double elevatorGearing = 20;
        public static final double elevatorWeightKG = Units.lbsToKilograms(40);
        public static final double spoolDiameter = Units.inchesToMeters(1.273);
        public static final double maxHeight = Units.inchesToMeters(21);
        public static final double rotationsPerMeter = elevatorGearing / spoolDiameter;

        public static final double elevatorTolerance = 0.02;

        public static final double kS = 0;
        public static final double kG = 0.29/12;
        public static final double kV = 20/12;
        public static final double kA = 0.08/12;
        
        public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);

        public static final double kP = 2;
        public static final double kI = 0.5;
        public static final double kD = 0;
        public static final double maxVelocity = 0.75;
        public static final double maxAcceleration = 1.75;

        public static final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final TrapezoidProfile.Constraints PIDConstraints = new Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController elevatorPID = new ProfiledPIDController(kP, kI, kD, PIDConstraints);

        //TODO: find setpoints for elevator
        public static enum ElevatorState{
            STOW(Units.inchesToMeters(0)),
            INTAKE(Units.inchesToMeters(1)),
            L1(Units.inchesToMeters(6)),
            L2(Units.inchesToMeters(13)),
            L3(Units.inchesToMeters(18)),
            L4(Units.inchesToMeters(21));

            public double height;
            private ElevatorState(double height){
                this.height = height;
            }
        }
    }

    public static final class CoralManipulator{
        public static final int angleMotorID = 2;
        public static final int rollerMotorID = 3;
        public static final int beamBreakDIO = 0;

        public static final double gearRatio = 5;
        public static final double MOI = 0.1007651;
        public static final double lengthMeters = 0.405;


        public static final double kS = 0;
        public static final double kG = 2.45/12;
        public static final double kV = 0.08/12;
        public static final double kA = 0.15/12;

        public static final ArmFeedforward angleFF = new ArmFeedforward(kS, kG, kV, kA);

        public static final double kP = 0.7;
        public static final double kI = 0.003;
        public static final double kD = 0.01;
        public static final double maxVelocity = 5;
        public static final double maxAcceleration = 3;

        public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController anglePID = new ProfiledPIDController(kP, kI, kD, constraints);

        public static enum CoralManipulatorPivotState{
            STOW(0),
            INTAKE(Units.degreesToRadians(45)),
            TROUGH(Units.degreesToRadians(225)),
            L2AND3(Units.degreesToRadians(235)),
            L4SETUP(Units.degreesToRadians(250)),
            L4FINAL(Units.degreesToRadians(220));

            public double angle;
            private CoralManipulatorPivotState(double angle){
                this.angle = angle;
            }
        }

        public static enum CoralManipulatorRollerState{
            OFF(0),
            INTAKE(0.3),
            OUTTAKE(-0.3);

            public double angle;
            private CoralManipulatorRollerState(double angle){
                this.angle = angle;
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
