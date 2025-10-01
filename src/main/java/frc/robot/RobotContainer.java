// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Deepclimb;
//import frc.robot.subsystems.ElevatorArmSimulation;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.commands.AutoAlgaeRemoval;
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

    //controllers
    //private final CommandJoystick driveStick = new CommandJoystick(0);
    //private final CommandJoystick secondaryStick = new CommandJoystick(1);
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final CommandPS4Controller ps4Controller = new CommandPS4Controller(2);

    //triggers
    //driver auto functions
    private final Trigger autoAlignLeftTrigger = driveController.leftBumper();//driveStick.button(11).or(ps4Controller.square());
    private final Trigger autoAlignRightTrigger = driveController.rightBumper();//driveStick.button(12).or(ps4Controller.circle());
    private final Trigger autoAlgaeRemovalTrigger = driveController.rightTrigger(0.1);//.button(14).or(ps4Controller.L1());

    //driver manual robot relative align
    private final Trigger manualAlignLeft = driveController.povLeft();
    private final Trigger manualAlignDown = driveController.povDown();
    private final Trigger manualAlignRight = driveController.povRight();
    private final Trigger manualAlignUp = driveController.povUp();

    //driver util
    private final Trigger forceVisionTrigger = driveController.x();//driveStick.button(13);
    private final Trigger zeroTrigger = driveController.a();
    private final Trigger brakeTrigger = driveController.b();

    //coral
    private final Trigger outtakeTrigger = secondaryController.rightTrigger(0.1);//rightTrigger(0.1);//secondaryStick.button(1);
    private final Trigger intakeTrigger = secondaryController.rightBumper();//secondaryStick.button(2);
    private final Trigger outtakeTroughTrigger = secondaryController.povUp();//secondaryStick.button(10);
    private final Trigger AlgaeRemovalTrigger = secondaryController.a();//secondaryStick.button(9);
    private final Trigger l1Trigger = secondaryController.povLeft();//secondaryStick.button(8);
    private final Trigger l2Trigger = secondaryController.x();//secondaryStick.button(5);
    private final Trigger l3Trigger = secondaryController.y();//secondaryStick.button(6);
    private final Trigger l4Trigger = secondaryController.b();//secondaryStick.button(7);
    //private final Trigger AlgaeRemovalSetup = secondaryStick.button(8);

    //algae
    private final Trigger intakeAlgaeTrigger = secondaryController.leftBumper();
    private final Trigger outtakeAlgaeTrigger = secondaryController.leftTrigger(0.1);

    //util
    private final Trigger zeroElevatorTrigger = secondaryController.povDown();//secondaryStick.button(16);

    //subsystems
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Elevator elevator = new Elevator();
    private final CoralManipulator coralManipulator = new CoralManipulator();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Deepclimb deepclimb = new Deepclimb();
    private final LED led = new LED();
    public final Vision vision = new Vision(drivetrain);

    //simulation
    //private final ElevatorArmSimulation elevatorArmSimulation = new ElevatorArmSimulation(elevator, coralManipulatorPivot);

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
        //teleop drive command
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //drivetrain.new DriveJoystick(driveStick)
            drivetrain.new DriveXbox(driveController)
            //SIM ONLY
            //drivetrain.new DrivePS4(ps4Controller)
        );

        //button bindings
        //driver auto
        autoAlignLeftTrigger.whileTrue(new AutoAlign(drivetrain, elevator, led, true));
        autoAlignRightTrigger.whileTrue(new AutoAlign(drivetrain, elevator, led, false));
        autoAlgaeRemovalTrigger.whileTrue(new AutoAlgaeRemoval(drivetrain, elevator, coralManipulator, led));

        //driver manual robot relative
        manualAlignLeft.whileTrue(drivetrain.new ManualAlign(0, 0.15));
        manualAlignDown.whileTrue(drivetrain.new ManualAlign(-0.15, 0));
        manualAlignRight.whileTrue(drivetrain.new ManualAlign(0, -0.15));
        manualAlignUp.whileTrue(drivetrain.new ManualAlign(0.15, 0));

        //driver util
        forceVisionTrigger.whileTrue(new InstantCommand(() -> vision.forceVision()));
        zeroTrigger.onTrue(new InstantCommand(() -> drivetrain.zero()));
        brakeTrigger.whileTrue(drivetrain.applyRequest(() -> brake));

        //drivetrain logging
        drivetrain.registerTelemetry(logger::telemeterize);

        //coral
        outtakeTrigger.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        outtakeTroughTrigger.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKETROUGH)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        intakeTrigger.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.rainbow()),
            elevator.new ChangeState(ElevatorState.INTAKE),
            coralManipulator.new ChangeState(CoralManipulatorPivotState.INTAKE, CoralManipulatorRollerState.INTAKE)))
            .onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        beamBreakLEDTrigger.onTrue(led.new BlinkLED(Color.kGreen, 0.25, 1, true));
        l1Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(ElevatorState.L1),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L1)));
        l2Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(ElevatorState.L2),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L2)));
        l3Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kBlue)),
            elevator.new ChangeState(ElevatorState.L3),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3)));
        l4Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kPurple)),
            elevator.new ChangeState(ElevatorState.L4),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4)));
        // AlgaeRemovalSetup.onTrue(new ParallelCommandGroup(
        //     coralManipulator.new ChangeState(CoralManipulatorPivotState.L3, CoralManipulatorRollerState.OUTTAKE),
        //     elevator.new ChangeState(ElevatorState.L3)));
        AlgaeRemovalTrigger.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangeState(CoralManipulatorPivotState.ALGAEREMOVAL, CoralManipulatorRollerState.OUTTAKE),
            elevator.new ChangeState(ElevatorState.L3)));

        //algae

        // old manip code
        // intakeAlgaeTrigger.whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> led.light(Color.kTeal)),
        //         algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.INTAKE)));
        // outtakeAlgaeTrigger.whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> led.light(Color.kTeal)),
        //         algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.OUTTAKE, true)));
        

        
        intakeAlgaeTrigger.whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                coralManipulator.new ChangePivotState(Constants.CoralManipulator.CoralManipulatorPivotState.ALGAEDEPLOY),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.INTAKE)))
        .onFalse(
            coralManipulator.new ChangePivotState(Constants.CoralManipulator.CoralManipulatorPivotState.STOW));

        outtakeAlgaeTrigger.whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                coralManipulator.new ChangePivotState(Constants.CoralManipulator.CoralManipulatorPivotState.ALGAEDEPLOY),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.OUTTAKE)))
        .onFalse( 
            new SequentialCommandGroup(
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.STOW),
                coralManipulator.new ChangePivotState(Constants.CoralManipulator.CoralManipulatorPivotState.STOW)));

        //util
        zeroElevatorTrigger.onTrue(new InstantCommand(() -> elevator.toggleZeroElevator())).onFalse(new InstantCommand(() -> elevator.toggleZeroElevator()));

        //deep climb :(
        // secondaryStick.button(15).whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> led.light(Color.kRed)),
        //         deepclimb.new ChangeState(Constants.Deepclimb.deepClimbStates.FORWARD)));
        // secondaryStick.button(14).whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> led.light(Color.kRed)),
        //         deepclimb.new ChangeState(Constants.Deepclimb.deepClimbStates.BACKWARD)));
    }

    public Command getAutonomousCommand() {
        drivetrain.resetPose(new Pose2d());
        /* Run the routine selected from the auto chooser */
        return autoRoutines.getGeneratedRoutine().cmd();
    }
}
