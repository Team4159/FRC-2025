// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //joysticks
    private final CommandJoystick driveStick = new CommandJoystick(0);
    private final CommandJoystick secondaryStick = new CommandJoystick(1);

    //triggers
    private final Trigger outtake = secondaryStick.button(1);
    private final Trigger intake = secondaryStick.button(2);
    private final Trigger AlgaeRemovalSetup = secondaryStick.button(8);
    private final Trigger AlgaeRemoval = secondaryStick.button(9);
    private final Trigger l2 = secondaryStick.button(5);
    private final Trigger l3 = secondaryStick.button(6);
    private final Trigger l4 = secondaryStick.button(7);
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
            drivetrain.new Drive(driveStick)
        );

        driveStick.pov(0).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0.15, 0));
        driveStick.pov(90).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, -0.15));
        driveStick.pov(180).whileTrue(drivetrain.new ManualAlign(secondaryStick, -0.15, 0));
        driveStick.pov(270).whileTrue(drivetrain.new ManualAlign(secondaryStick, 0, 0.15));

        autoAlignLeft.whileTrue(new AutoAlign(drivetrain, led, true));
        autoAlignRight.whileTrue(new AutoAlign(drivetrain, led, false));
        forceVision.whileTrue(new InstantCommand(() -> vision.forceVision()));
        driveStick.button(1).onTrue(new InstantCommand(() -> drivetrain.zero()));
        driveStick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);

        outtake.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        outtakeTrough.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKETROUGH)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        intake.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.rainbow()),
            elevator.new ChangeState(ElevatorState.INTAKE),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.INTAKE),
            coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.INTAKE)))
            .onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        beamBreakLEDTrigger.onTrue(led.new BlinkLED(Color.kGreen, 0.25, 1, true));
        l2.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(ElevatorState.L2),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L2)));
        l3.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kBlue)),
            elevator.new ChangeState(ElevatorState.L3),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3)));
        l4.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kPurple)),
            elevator.new ChangeState(ElevatorState.L4),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4)));
        AlgaeRemovalSetup.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3),
            elevator.new ChangeState(ElevatorState.L3),
            coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)));
        AlgaeRemoval.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.ALGAEREMOVAL),
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
    }

    public Command getAutonomousCommand() {
        drivetrain.resetPose(new Pose2d());
        /* Run the routine selected from the auto chooser */
        //return autoChooser.selectedCommand();
        return autoRoutines.getRoutine().cmd();
    }
}
