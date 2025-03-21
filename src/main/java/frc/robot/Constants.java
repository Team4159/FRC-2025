package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
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
        public static final int distanceSensorPingDIO = 0;
        public static final int distanceSensorEchoDIO = 1;
        //accounts for dist sensor is located behind bumper
        public static final double distSensorAutoAlignDistInches = 6;
        //bumpers included
        /** Units: meters */
        public static final double width = Units.inchesToMeters(33);
        public static final double distFromReefBase = Units.inchesToMeters(2);
        /** Units: meters */
        public static final double L4Offset = Units.inchesToMeters(3.5);
        /** Units: meters */
        public static final double maxReefAutoAlignDistatnce = Units.inchesToMeters(120);

        //autoaim
        public static final  TrapezoidProfile.Constraints translationConstraints = new Constraints(0.75, 0.5);
        public static final  TrapezoidProfile.Constraints rotationConstraints = new Constraints(3, 3);
        public static final ProfiledPIDController translationController = new ProfiledPIDController(2, 0.5, 0, translationConstraints);
        public static final ProfiledPIDController rotationController = new ProfiledPIDController(7, 0.1, 0.1, rotationConstraints){{
            enableContinuousInput(-Math.PI, Math.PI);
        }};

        public static final HolonomicDriveController holonomicController= new HolonomicDriveController(
            new PIDController(2, 0, 0),
            new PIDController(2, 0, 0),
            new ProfiledPIDController(5, 0, 0,
                    new TrapezoidProfile.Constraints(2, 2)));

        public static final double maxAccelFullExtension = 2;
        public static final double maxAccelFullRetraction = 3;
    }

    public static final class Deepclimb {
        public static enum deepClimbStates {
            FORWARD(1),
            BACKWARD(-1);

            public final double speed;        
             
            // take enum paremter that is deep climb states, set motor speed to state.speed
            private deepClimbStates(double s){
                speed = s;
            }

            // public double getSpeed_deepClimb(){
            //     return speed;
            // }
        }
        
        
        public static final double forwardSoftLimit = 10;
        public static final int deviceID = 7;
        public static final double reverseSoftLimit = 0;
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

        public static final double elevatorTolerance = 0.0025;

        public static final double kS = 0.15;
        //factors in cf spring
        public static final double kG = 0.04;//0.09 * 1.6;
        public static final double kV = 26.12;//26.12 * 1.6;//15.67/12;
        public static final double kA = 0.01;//0.02 * 1.6;//0.07/12;
        
        public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);

        public static final double kP = 80;
        public static final double kI = 12;
        public static final double kD = 0;
        public static final double maxVelocity = 10;
        public static final double maxAcceleration = 10;

        //public static final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final TrapezoidProfile.Constraints PIDConstraints = new Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController elevatorPID = new ProfiledPIDController(kP, kI, kD, PIDConstraints);

        //TODO: find setpoints for elevator
        public static enum ElevatorState{
            STOW(Units.inchesToMeters(0)),
            INTAKE(Units.inchesToMeters(0)),
            L1(Units.inchesToMeters(2)),
            L2(Units.inchesToMeters(2)),
            L3(Units.inchesToMeters(7.3)),
            L4(Units.inchesToMeters(21.5));

            public double height;
            private ElevatorState(double height){
                this.height = height;
            }
        }
    }

    public static final class CoralManipulator{
        public static final int angleMotorID = 4;
        public static final int rollerMotorID = 5;
        public static final int beamBreakDIO = 9;

        public static final double gearRatio = 60;
        public static final double MOI = 0.2466;
        public static final double lengthMeters = 0.405;

        public static final double kSEmpty = 0;
        public static final double kGEmpty = 0.3;//0.34;//0.8/12;//1.45/12;//2.45/12;
        public static final double kVEmpty = 1.01*3/4;//1.01*2/3;//0.76/12;//0.42/12;//0.08/12;
        public static final double kAEmpty = 0.02;//0.03/12;//0.05/12;//0.15/12;

        public static final double kSCoral = 0;
        public static final double kGCoral = 0.35;//0.34;//0.8/12;//1.45/12;//2.45/12;
        public static final double kVCoral = 1.01*3/4;//1.01*2/3;//0.76/12;//0.42/12;//0.08/12;
        public static final double kACoral = 0.02;//0.03/12;//0.05/12;//0.15/12;

        public static final ArmFeedforward angleFFEmpty = new ArmFeedforward(kSEmpty, kGEmpty, kVEmpty, kAEmpty);
        public static final ArmFeedforward angleFFCoral = new ArmFeedforward(kSCoral, kGCoral, kVCoral, kACoral);

        public static final double kP = 2;
        public static final double kI = 0.05;
        public static final double kD = 0;
        public static final double maxVelocity = 7;
        public static final double maxAcceleration = 7;

        public static final double FFOffset = Units.rotationsToRadians(-0.1);

        public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        public static final ProfiledPIDController anglePID = new ProfiledPIDController(kP, kI, kD, constraints);

        public static final double angleTolerance = Math.PI/16;

        public static enum CoralManipulatorPivotState{
            STOW(0),
            INTAKE(Units.degreesToRadians(0)),
            ALGAEREMOVAL(Units.degreesToRadians(150)),
            TROUGH(Units.degreesToRadians(200)),
            L2(Units.degreesToRadians(200)),
            L3(Units.degreesToRadians(190)),
            L4(Units.degreesToRadians(196));

            public double angle;
            private CoralManipulatorPivotState(double angle){
                this.angle = angle;
            }
        }

        public static enum CoralManipulatorRollerState{
            PASSIVE(0.02),
            INTAKE(1),
            OUTTAKE(-1),
            OUTTAKETROUGH(-0.2);

            public double spin;
            private CoralManipulatorRollerState(double spin){
                this.spin = spin;
            }
        }
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
            INTAKE(0.5, 1),
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
            new Pose2d(fieldLength/2 - Units.inchesToMeters(183.259) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(2*Math.PI/3), fieldWidth/2 + Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(2*Math.PI/3), new Rotation2d(-Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(199.431) - (poleDist + Swerve.distFromReefBase + Swerve.width/2), fieldWidth/2, new Rotation2d()),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(183.259) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(-2*Math.PI/3), fieldWidth/2 - Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(-2*Math.PI/3), new Rotation2d(Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(153.728) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(-Math.PI/3), fieldWidth/2 - Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(-Math.PI/3), new Rotation2d(2*Math.PI/3)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(137.999) + (poleDist + Swerve.distFromReefBase + Swerve.width/2), fieldWidth/2, new Rotation2d(Math.PI)),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(153.728) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(Math.PI/3), fieldWidth/2 + Units.inchesToMeters(26.487) + (poleDist+ Swerve.distFromReefBase + Swerve.width/2)*Math.sin(Math.PI/3), new Rotation2d(-2*Math.PI/3))
        );

        public static final List<Pose2d> reefRed = List.of(
            new Pose2d(fieldLength/2 + Units.inchesToMeters(183.259) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(Math.PI/3), fieldWidth/2  + Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(Math.PI/3), new Rotation2d(-2*Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(199.431) + (poleDist + Swerve.distFromReefBase + Swerve.width/2), fieldWidth/2, new Rotation2d(Math.PI)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(183.259) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(-Math.PI/3), fieldWidth/2  - Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(-Math.PI/3), new Rotation2d(2*Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(153.728) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(-2*Math.PI/3), fieldWidth/2  - Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(-2*Math.PI/3), new Rotation2d(Math.PI/3)),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(137.999) - (poleDist + Swerve.distFromReefBase + Swerve.width/2), fieldWidth/2, new Rotation2d()),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(153.728) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.cos(2*Math.PI/3), fieldWidth/2  + Units.inchesToMeters(26.487) + (poleDist + Swerve.distFromReefBase + Swerve.width/2)*Math.sin(2*Math.PI/3), new Rotation2d(-Math.PI/3))
        );

        public static final List<Pose2d> coralStationsBlue = List.of(
            new Pose2d(fieldLength/2 - Units.inchesToMeters(331.175) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(119.416) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(312.767) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(133.519) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(292.336) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(147.622) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 - 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(331.175) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(119.416) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(312.767) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(133.519) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899))),
            new Pose2d(fieldLength/2 - Units.inchesToMeters(292.336) + (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(147.622) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 + 35.899)))
        );

        public static final List<Pose2d> coralStationsRed = List.of(
            new Pose2d(fieldLength/2 + Units.inchesToMeters(331.175) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(119.416) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(312.767) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(133.519) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(292.336) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 - Units.inchesToMeters(147.622) + (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(90 + 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(331.175) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(119.416) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(312.767) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(133.519) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899))),
            new Pose2d(fieldLength/2 + Units.inchesToMeters(292.336) - (Swerve.width/2) * Math.sin(Units.degreesToRadians(35.899)), fieldWidth/2 + Units.inchesToMeters(147.622) - (Swerve.width/2) * Math.cos(Units.degreesToRadians(35.899)), new Rotation2d(Units.degreesToRadians(-90 - 35.899)))
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
