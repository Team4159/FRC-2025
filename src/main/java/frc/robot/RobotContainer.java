// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorArmSimulation;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.Deepclimb;
//import frc.robot.subsystems.ElevatorArmSimulation;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoOuttake;
import frc.robot.commands.AutoSwerve;
import frc.robot.commands.TrajSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulatorPivot;
//import frc.robot.subsystems.CoralManipulatorPivot;
import frc.robot.subsystems.CoralManipulatorRoller;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity 0.75 old value

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //ps4 controller for simulation at home
    private final CommandPS4Controller joystick = new CommandPS4Controller(2);

    //joysticks
    private final CommandJoystick driveStick = new CommandJoystick(0);

    private final CommandJoystick secondaryStick = new CommandJoystick(1);

    //triggers
    private final Trigger outtake = secondaryStick.button(1).or(joystick.L2());
    private final Trigger intake = secondaryStick.button(2).or(joystick.R2());
    //private final Trigger l1 = secondaryStick.button(8).or(joystick.cross());
    private final Trigger AlgaeRemovalSetup = secondaryStick.button(8);
    private final Trigger AlgaeRemoval = secondaryStick.button(9);
    private final Trigger l2 = secondaryStick.button(5).or(joystick.square());
    private final Trigger l3 = secondaryStick.button(6).or(joystick.circle());
    private final Trigger l4 = secondaryStick.button(7).or(joystick.triangle());
    private final Trigger raiseElevator = secondaryStick.button(3).or(joystick.L1());
    private final Trigger forceVision = driveStick.button(3);

    private final Trigger zeroELevator = secondaryStick.button(16);

    //subsystems

    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Elevator elevator = new Elevator();
    private final CoralManipulatorPivot coralManipulatorPivot = new CoralManipulatorPivot();
    private final CoralManipulatorRoller coralManipulatorRoller = new CoralManipulatorRoller();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Deepclimb deepclimb = new Deepclimb();
    private final LED led = new LED();

    //simulation
    private final ElevatorArmSimulation elevatorArmSimulation = new ElevatorArmSimulation(elevator, coralManipulatorPivot);
    public final Vision vision = new Vision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, elevator, coralManipulatorPivot, coralManipulatorRoller);

        drivetrain.setElevator(elevator);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("C42Coral", autoRoutines::C42Coral);
        autoChooser.addRoutine("M2 3coral", autoRoutines::M23coral);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //PS4 controller for simulation
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )

            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-driveStick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-driveStick.getX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-driveStick.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            //new InstantCommand(() -> drivetrain.drive(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX()))
            drivetrain.new Drive(driveStick)

            //new InstantCommand(() -> drivetrain.drive(-driveStick.getY(), -driveStick.getX(), -driveStick.getZ))
        );

        //PS4 controller

        // joystick.square().whileTrue(new AutoAlign(drivetrain));
        // joystick.triangle().whileTrue(new AutoAlign(drivetrain, true));
        // //joystick.square().whileTrue(drivetrain.getAutoAlignCommand());
        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // driveStick.pov(0).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0.15, 0));
        // driveStick.pov(90).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, -0.15));
        // driveStick.pov(180).whileTrue(drivetrain.new ManualAlign(secondaryStick, -0.15, 0));
        // driveStick.pov(270).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, 0.15));

        // driveStick.pov(0).or(driveStick.pov(180)).whileTrue(new ManualDSAlign(drivetrain, 0));
        // driveStick.pov(90).whileTrue(new ManualDSAlign(drivetrain, -0.15));
        // driveStick.pov(270).whileTrue(new ManualDSAlign(drivetrain, 0.15));

        driveStick.button(11).whileTrue(new AutoAlign(drivetrain));
        driveStick.button(12).whileTrue(new AutoAlign(drivetrain, false, true));
        driveStick.button(13).whileTrue(new InstantCommand(() -> vision.forceVision()));
        //driveStick.button(6).whileTrue(new AutoAlign(drivetrain, true));
        //driveStick.button(3).onTrue(new InstantCommand(() -> vision.forceVision()));
        driveStick.button(1).onTrue(new InstantCommand(() -> drivetrain.zero()));
        //joystick.square().whileTrue(drivetrain.getAutoAlignCommand());
        driveStick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));
        driveStick.button(16).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Note that each routine should be run exactly once in a single log.
        // driveStick.button(5).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveStick.button(6).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveStick.button(7).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveStick.button(8).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.L2().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //forceVision.onTrue(drivetrain.forceVision());

        drivetrain.registerTelemetry(logger::telemeterize);

        outtake.onTrue(coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.OUTTAKE)).onFalse(coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.PASSIVE));
        //intake.onTrue(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator, true));
        intake.onTrue(new ParallelCommandGroup(
            led.new ChromaLED((double i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly(),
            elevator.new ChangeState(ElevatorState.INTAKE, true),
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.INTAKE, false),
            coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.INTAKE)))
            .onFalse(coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.PASSIVE));
        l2.onTrue(new ParallelCommandGroup(
            led.new LightLED(0, 255, 0),
            elevator.new ChangeState(ElevatorState.L2, false),
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L2, false)));
            //new InstantCommand(() -> elevator.setFutureState(ElevatorState.L2))));
        l3.onTrue(new ParallelCommandGroup(
            led.new LightLED(0, 0, 255),
            elevator.new ChangeState(ElevatorState.L3, false),
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L3, false)));
            //new InstantCommand(() -> elevator.setFutureState(ElevatorState.L3))));
        l4.onTrue(new ParallelCommandGroup(
            led.new LightLED(70, 0, 100),
            elevator.new ChangeState(ElevatorState.L4, false),
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false)));
            //new InstantCommand(() -> elevator.setFutureState(ElevatorState.L4))));
        AlgaeRemovalSetup.onTrue(new ParallelCommandGroup(
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L3, false),
            elevator.new ChangeState(ElevatorState.L3, false),
            coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.OUTTAKE)));
        AlgaeRemoval.onTrue(new ParallelCommandGroup(
            coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.ALGAEREMOVAL, false),
            coralManipulatorRoller.new ChangeState(CoralManipulatorRollerState.OUTTAKE)));
        raiseElevator.onTrue(new InstantCommand(() -> elevator.goToFutureState()));

        secondaryStick.button(12).whileTrue(
            new ParallelCommandGroup(
                led.new LightLED(50, 210, 200),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.INTAKE)));
        secondaryStick.button(13).whileTrue(
            new ParallelCommandGroup(
                led.new LightLED(50, 210, 200),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.OUTTAKE, true)));

        zeroELevator.onTrue(new InstantCommand(() -> elevator.toggleZeroElevator())).onFalse(new InstantCommand(() -> elevator.toggleZeroElevator()));

        secondaryStick.button(15).whileTrue(
            new ParallelCommandGroup(
                led.new LightLED(255, 0, 0),
                deepclimb.new ChangeState(Constants.Deepclimb.deepClimbStates.FORWARD)));
        secondaryStick.button(14).whileTrue(
            new ParallelCommandGroup(
                led.new LightLED(255, 0, 0),
                deepclimb.new ChangeState(Constants.Deepclimb.deepClimbStates.BACKWARD)));

        // led.bindButton(intake, LEDState.RAINBOW);
        // led.bindButton(secondaryStick.button(12).or(secondaryStick.button(13)), LEDState.TURQUOISE);
        // led.bindButton(l4, LEDState.PURPLE);
        // led.bindButton(l3, LEDState.BLUE);
        // led.bindButton(l2, LEDState.GREEN);
        // led.bindButton(l1, LEDState.WHITE);
        // led.bindButton(secondaryStick.button(15).or(secondaryStick.button(14)), LEDState.RED);
    }

    public Command getAutonomousCommand() {
        drivetrain.resetPose(new Pose2d());
        /* Run the routine selected from the auto chooser */
        //return autoChooser.selectedCommand();
        return autoRoutines.getRoutine().cmd();

        //Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<Translation2d>(), new Pose2d(1, 0, new Rotation2d()), new TrajectoryConfig(1, 1));
        //return new TrajSwerve(drivetrain, traj);
    }
}
