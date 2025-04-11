// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Deepclimb;
//import frc.robot.subsystems.ElevatorArmSimulation;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.commands.AutoAlign;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity 0.75 old value

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //ps4 controller for simulation at home
    //private final CommandPS4Controller joystick = new CommandPS4Controller(2);

    //joysticks
    private final CommandJoystick driveStick = new CommandJoystick(0);

    private final CommandJoystick secondaryStick = new CommandJoystick(1);

    //triggers
    private final Trigger outtake = secondaryStick.button(1);//.or(joystick.L2());
    private final Trigger intake = secondaryStick.button(2);//.or(joystick.R2());
    //private final Trigger l1 = secondaryStick.button(8).or(joystick.cross());
    private final Trigger AlgaeRemovalSetup = secondaryStick.button(8);
    private final Trigger AlgaeRemoval = secondaryStick.button(9);
    private final Trigger l2 = secondaryStick.button(5);//.or(joystick.square());
    private final Trigger l3 = secondaryStick.button(6);//.or(joystick.circle());
    private final Trigger l4 = secondaryStick.button(7);//.or(joystick.triangle());
    private final Trigger forceVision = driveStick.button(13);
    private final Trigger autoAlignLeft = driveStick.button(11);
    private final Trigger autoAlignRight = driveStick.button(12);

    private final Trigger outtakeTrough = secondaryStick.button(10);

    private final Trigger zeroELevator = secondaryStick.button(16);

    //subsystems

    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Elevator elevator = new Elevator();
    private final CoralManipulator coralManipulator = new CoralManipulator();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Deepclimb deepclimb = new Deepclimb();
    private final LED led = new LED();

    //simulation
    //private final ElevatorArmSimulation elevatorArmSimulation = new ElevatorArmSimulation(elevator, coralManipulatorPivot);
    public final Vision vision = new Vision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;

    private final Trigger beamBreakLEDTrigger = new Trigger(coralManipulator :: hasCoral);

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, elevator, coralManipulator, led);

        drivetrain.setElevator(elevator);

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
            drivetrain.new Drive(driveStick)
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

        driveStick.pov(0).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0.15, 0));
        driveStick.pov(90).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, -0.15));
        driveStick.pov(180).whileTrue(drivetrain.new ManualAlign(secondaryStick, -0.15, 0));
        driveStick.pov(270).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, 0.15));

        autoAlignLeft.whileTrue(new AutoAlign(drivetrain, led, true));
        autoAlignRight.whileTrue(new AutoAlign(drivetrain, led, false));
        forceVision.whileTrue(new InstantCommand(() -> vision.forceVision()));
        driveStick.button(1).onTrue(new InstantCommand(() -> drivetrain.zero()));
        //joystick.square().whileTrue(drivetrain.getAutoAlignCommand());
        driveStick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));

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
        // joystick.L2().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //forceVision.onTrue(drivetrain.forceVision());

        drivetrain.registerTelemetry(logger::telemeterize);

        outtake.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        outtakeTrough.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKETROUGH)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        //intake.onTrue(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator, true));
        intake.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.rainbow()),
            elevator.new ChangeState(ElevatorState.INTAKE, true),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.INTAKE, false),
            coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.INTAKE)))
            .onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        beamBreakLEDTrigger.onTrue(led.new BlinkLED(Color.kGreen, 0.25, 1, true));
        l2.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(ElevatorState.L2, false),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L2, false)));
        l3.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kBlue)),
            elevator.new ChangeState(ElevatorState.L3, false),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3, false)));
        l4.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kPurple)),
            elevator.new ChangeState(ElevatorState.L4, false),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4, false)));
        AlgaeRemovalSetup.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3, false),
            elevator.new ChangeState(ElevatorState.L3, false),
            coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)));
        AlgaeRemoval.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.ALGAEREMOVAL, false),
            coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)));

        secondaryStick.button(12).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.INTAKE)));
        secondaryStick.button(13).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.OUTTAKE, true)));

        zeroELevator.onTrue(new InstantCommand(() -> elevator.toggleZeroElevator())).onFalse(new InstantCommand(() -> elevator.toggleZeroElevator()));

        secondaryStick.button(15).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kRed)),
                deepclimb.new ChangeState(Constants.Deepclimb.deepClimbStates.FORWARD)));
        secondaryStick.button(14).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kRed)),
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
    }
}
