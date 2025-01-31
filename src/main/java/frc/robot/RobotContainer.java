// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorArmSimulation;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulatorPivot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(2);

    private final CommandJoystick driveStick = new CommandJoystick(0);
    private final CommandJoystick secondaryStick = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final CoralManipulatorPivot coralManipulatorPivot = new CoralManipulatorPivot();

    private final ElevatorArmSimulation elevatorArmSimulation = new ElevatorArmSimulation(elevator, coralManipulatorPivot);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("C42Coral", autoRoutines::C42Coral);
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

            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveStick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveStick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveStick.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
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

        driveStick.button(5).whileTrue(new AutoAlign(drivetrain));
        driveStick.button(6).whileTrue(new AutoAlign(drivetrain, true));
        driveStick.button(1).onTrue(new InstantCommand(() -> drivetrain.zero()));
        //joystick.square().whileTrue(drivetrain.getAutoAlignCommand());
        driveStick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));
        driveStick.button(16).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.L2().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        secondaryStick.button(2).onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.INTAKE),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE)));
        secondaryStick.button(5).onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L2),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L2AND3)));
        secondaryStick.button(6).onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L3),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L2AND3)));
        secondaryStick.button(7).onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L4),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L4SETUP)));

        //PS4 controller
        joystick.cross().onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.INTAKE),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE)));
        joystick.square().onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L2),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L2AND3)));
        joystick.circle().onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L3),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L2AND3)));
        joystick.triangle().onTrue(new ParallelCommandGroup(
            elevator.new ChangeState(Constants.Elevator.ElevatorState.L4),
            coralManipulatorPivot.new ChangeState(Constants.CoralManipulator.CoralManipulatorPivotState.L4SETUP)));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
