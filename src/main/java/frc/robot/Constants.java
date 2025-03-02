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
        /** Units: meters */
        public static final double width = Units.inchesToMeters(33);
        /** Units: meters */
        public static final double L4Offset = Units.inchesToMeters(3.5);
        /** Units: meters */
        public static final double maxReefAutoAlignDistatnce = Units.inchesToMeters(120);

        //autoaim
        public static final  TrapezoidProfile.Constraints translationConstraints = new Constraints(0.25, 0.25);
        public static final  TrapezoidProfile.Constraints rotationConstraints = new Constraints(2, 2);
        public static final ProfiledPIDController translationController = new ProfiledPIDController(0.5, 0.1, 0, translationConstraints);
        public static final ProfiledPIDController rotationController = new ProfiledPIDController(10, 0.1, 0.1, rotationConstraints){{
            enableContinuousInput(-Math.PI, Math.PI);
        }};

        public static final double maxAccelFullExtension = 0.75;
        public static final double maxAccelFullRetraction = 3;
    }

    public static final class Elevator{
        public static final int elevatorMotorID = 6;
        public static final int limitSwitchPort = 0;

        public static final double elevatorGearing = 25;
        public static final double elevatorWeightKG = Units.lbsToKilograms(20);
        public static final double spoolDiameter = Units.inchesToMeters(1.273);
        public static final double maxHeight = Units.inchesToMeters(21);
        public static final double rotationsPerMeter = elevatorGearing / (Math.PI*spoolDiameter);

        //public static final double zeroModeThreshold = 0;

        public static final double elevatorTolerance = 0.02;

        public static final double kS = 0.15;
        public static final double kG = 0.09 * 1.6;
        public static final double kV = 26.12 * 1.6;//15.67/12;
        public static final double kA = 0.02 * 1.6;//0.07/12;
        
        public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);

        public static final double kP = 70;
        public static final double kI = 5;
        public static final double kD = 0;
        public static final double maxVelocity = 1000;
        public static final double maxAcceleration = 30;

        //public static final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final TrapezoidProfile.Constraints PIDConstraints = new Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController elevatorPID = new ProfiledPIDController(kP, kI, kD, PIDConstraints);

        //TODO: find setpoints for elevator
        public static enum ElevatorState{
            STOW(Units.inchesToMeters(0)),
            INTAKE(Units.inchesToMeters(0)),
            L1(Units.inchesToMeters(3)),
            L2(Units.inchesToMeters(3)),
            L3(Units.inchesToMeters(10)),
            L4(Units.inchesToMeters(22));

            public double height;
            private ElevatorState(double height){
                this.height = height;
            }
        }
    }

    public static final class CoralManipulator{
        public static final int angleMotorID = 14;//4
        public static final int rollerMotorID = 5;
        public static final int beamBreakDIO = 9;

        public static final double gearRatio = 60;
        public static final double MOI = 0.2466;
        public static final double lengthMeters = 0.405;

        public static final double kS = 0.5;
        //kg = 0.17 for new coral manipulator
        public static final double kG = 0.34;//0.8/12;//1.45/12;//2.45/12;
        public static final double kV = 1.01*2/3;//0.76/12;//0.42/12;//0.08/12;
        public static final double kA = 0.02;//0.03/12;//0.05/12;//0.15/12;

        public static final ArmFeedforward angleFF = new ArmFeedforward(kS, kG, kV, kA);

        public static final double kP = 0.9;
        public static final double kI = 0.008;
        public static final double kD = 0;
        public static final double maxVelocity = 5;
        public static final double maxAcceleration = 5;

        public static final double FFOffset = Units.rotationsToRadians(-0.1);

        public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController anglePID = new ProfiledPIDController(kP, kI, kD, constraints);

        public static final double angleTolerance = Math.PI/64;

        public static enum CoralManipulatorPivotState{
            STOW(0),
            INTAKE(Units.degreesToRadians(20)),
            TROUGH(Units.degreesToRadians(200)),
            L2(Units.degreesToRadians(220)),
            L3(Units.degreesToRadians(220)),
            L4SETUP(Units.degreesToRadians(210)),
            L4FINAL(Units.degreesToRadians(210));

            public double angle;
            private CoralManipulatorPivotState(double angle){
                this.angle = angle;
            }
        }

        public static enum CoralManipulatorRollerState{
            OFF(0),
            INTAKE(1),
            OUTTAKE(-1);

            public double spin;
            private CoralManipulatorRollerState(double spin){
                this.spin = spin;
            }
        }
    }

    public static final class Field{
        /** Units: meters */
        public static final double poleDist = Units.inchesToMeters(1.652);
        /** Units: meters */
        public static final double fieldLength = Units.inchesToMeters(690.875);
        /** Units: meters */
        public static final double fieldWidth = Units.inchesToMeters(317);
        /** Units: meters */
        public static final double middletoPole = Units.inchesToMeters(12.938/2);
        /** Units: meters */
        public static final double reefDistFromCenter = Units.inchesToMeters(168.692);

        public static final List<Pose2d> reefBlue = List.of(
            new Pose2d(fieldLength/2 - Units.inchesToMeters(183.259) + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3), fieldWidth/2 + Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3), new Rotation2d(-Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(199.431) - (poleDist + Swerve.width/2), fieldWidth/2, new Rotation2d()),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(183.259) + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3), fieldWidth/2 - Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3), new Rotation2d(Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(153.728) + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3), fieldWidth/2 - Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3), new Rotation2d(2*Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(137.999) + (poleDist + Swerve.width/2), fieldWidth/2, new Rotation2d(Math.PI)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(153.728) + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3), fieldWidth/2 + Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3), new Rotation2d(-2*Math.PI/3))
        );

        public static final List<Pose2d> reefRed = List.of(
            new Pose2d(fieldLength/2 + Units.inchesToMeters(183.259) + (poleDist + Swerve.width/2)*Math.cos(Math.PI/3), fieldWidth/2  + Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(Math.PI/3), new Rotation2d(-2*Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(199.431) + (poleDist + Swerve.width/2), fieldWidth/2, new Rotation2d(Math.PI)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(183.259) + (poleDist + Swerve.width/2)*Math.cos(-Math.PI/3), fieldWidth/2  - Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(-Math.PI/3), new Rotation2d(2*Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(153.728) + (poleDist + Swerve.width/2)*Math.cos(-2*Math.PI/3), fieldWidth/2  - Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(-2*Math.PI/3), new Rotation2d(Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(137.999) - (poleDist + Swerve.width/2), fieldWidth/2, new Rotation2d()),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(153.728) + (poleDist + Swerve.width/2)*Math.cos(2*Math.PI/3), fieldWidth/2  + Units.inchesToMeters(26.487) + (poleDist + Swerve.width/2)*Math.sin(2*Math.PI/3), new Rotation2d(-Math.PI/3))
        );

        public static final List<Pose2d> coralStationsBlue = List.of(
            new Pose2d(fieldLength/2 - Units.inchesToMeters(331.175) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(119.416) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(312.767) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(133.519) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(292.336) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(147.622) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(331.175) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(119.416) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(312.767) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(133.519) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(292.336) + Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(147.622) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899)))
        );

        public static final List<Pose2d> coralStationsRed = List.of(
            new Pose2d(fieldLength/2 + Units.inchesToMeters(331.175) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(119.416) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(312.767) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(133.519) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(292.336) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(147.622) + Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(331.175) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(119.416) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(312.767) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(133.519) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(292.336) - Swerve.width/2 * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(147.622) - Swerve.width/2 * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899)))
        );

        public static final Map<DriverStation.Alliance, List<Pose2d>> reef = Map.of(
            Alliance.Blue, reefBlue,
            Alliance.Red, reefRed
        );

        public static final Map<DriverStation.Alliance, List<Pose2d>> stations = Map.of(
            Alliance.Blue, coralStationsBlue,
            Alliance.Red, coralStationsRed
        );

        public static final Map<DriverStation.Alliance, Pose2d> processors = Map.of(
            Alliance.Blue, new Pose2d(fieldLength/2 + Units.inchesToMeters(109.712), fieldWidth/2 + Units.inchesToMeters(158.455) - Constants.Swerve.width/2, new Rotation2d(-Math.PI/2)),
            Alliance.Red, new Pose2d(fieldLength/2 - Units.inchesToMeters(109.712), fieldWidth/2 - Units.inchesToMeters(158.455) + Constants.Swerve.width/2, new Rotation2d(Math.PI/2))
        );
    }
}
