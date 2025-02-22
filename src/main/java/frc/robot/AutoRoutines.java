package frc.robot;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoOuttake;
import frc.robot.subsystems.CoralManipulatorPivot;
import frc.robot.subsystems.CoralManipulatorRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines extends SubsystemBase{
    private final AutoFactory m_factory;
    private final Elevator elevator;
    private final CoralManipulatorPivot coralManipulatorPivot;
    private final CoralManipulatorRoller coralManipulatorRoller;
    private final CommandSwerveDrivetrain swerve;
    private SendableChooser<String> startChooser, reef1Chooser, station1Chooser, reef2Chooser, station2Chooser, reef3Chooser;
    private Field2d field2d;

    // private AutoRoutine routine;
    // private AutoTrajectory starttoR1;
    // private AutoTrajectory R1toS1;
    // private AutoTrajectory S1toR2;
    // private AutoTrajectory R2toS2;
    // private AutoTrajectory S2toR3;

    private boolean generatePreview;

    private edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain swerve, Elevator elevator, CoralManipulatorPivot coralManipulatorPivot, CoralManipulatorRoller coralManipulatorRoller) {
        m_factory = factory;
        this.elevator = elevator;
        this.coralManipulatorPivot = coralManipulatorPivot;
        this.coralManipulatorRoller = coralManipulatorRoller;
        this.swerve = swerve;
        field2d = new Field2d();
        instantiatePointChoosers();
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine C42Coral() {
        final AutoRoutine routine = m_factory.newRoutine("C4 2Coral");
        final AutoTrajectory C4toS2 = routine.trajectory("C4toS2");
        final AutoTrajectory S2toC3 = routine.trajectory("S2toC3");

        routine.active().onTrue(
            C4toS2.resetOdometry()
                .andThen(C4toS2.cmd())
                .andThen(S2toC3.cmd())
        );
        return routine;
    }

    public AutoRoutine M23coral(){
        final AutoRoutine routine = m_factory.newRoutine("M2 3coral");
        final AutoTrajectory M2toC10 = routine.trajectory("M2toC10");
        final AutoTrajectory C10toST1 = routine.trajectory("C10toST1");
        final AutoTrajectory ST1toC3 = routine.trajectory("ST1toC3");
        final AutoTrajectory C3toST1 = routine.trajectory("C3toST1");
        final AutoTrajectory ST1toC4 = routine.trajectory("ST1toC4");

        routine.active().onTrue(
            M2toC10.resetOdometry()
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    M2toC10.cmd()))
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
            .andThen(C10toST1.cmd())
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator))
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    ST1toC3.cmd()))
                    .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
            .andThen(C3toST1.cmd())
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator))
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    ST1toC4.cmd()))
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
        );
        return routine;
    }

    public AutoRoutine getRoutine(){
        final AutoRoutine routine = m_factory.newRoutine("routine");
        final String r1 = reef1Chooser.getSelected().substring(0, 2);
        final String r2 = reef2Chooser.getSelected().substring(0, 2);
        final String r3 = reef3Chooser.getSelected().substring(0, 2);
        final AutoTrajectory starttoR1 = routine.trajectory(startChooser.getSelected() + "to" + r1);
        final AutoTrajectory R1toS1 = routine.trajectory(r1 + "to" + station1Chooser.getSelected());
        final AutoTrajectory S1toR2 = routine.trajectory(station1Chooser.getSelected() + "to" + r2);
        final AutoTrajectory R2toS2 = routine.trajectory(r2 + "to" + station2Chooser.getSelected());
        final AutoTrajectory S2toR3 = routine.trajectory(station2Chooser.getSelected() + "to" + r3);
        
        routine.active().onTrue(
            starttoR1.resetOdometry()
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    starttoR1.cmd()))
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
            .andThen(R1toS1.cmd())
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator))
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    S1toR2.cmd()))
                    .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
            .andThen(R2toS2.cmd())
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator))
            .andThen(
                new ParallelCommandGroup(
                    coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4SETUP, false),
                    elevator.new ChangeState(ElevatorState.L4, false),
                    S2toR3.cmd()))
            .andThen(new InstantCommand(() -> swerve.stopSwerve()))
            .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
        );
        updateField(starttoR1, R1toS1, S1toR2, R2toS2, S2toR3);
        return routine;
    }

    // @Override
    // public void periodic(){
    //     getRoutine();
    // }

    public void updateField(AutoTrajectory... trajectories){
        trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        for(int i = 0; i < trajectories.length; i++){
            Trajectory<SwerveSample> choreoTrajectory = trajectories[i].getRawTrajectory();
            // field2d.getObject(traj.toString()+" traj").setPoses(
            // traj.getInitialPose().orElse(new Pose2d()), traj.getFinalPose().orElse(new Pose2d())
            // );
            // field2d.getObject(traj.toString()+" trajPoses").setPoses(
            //     t.getPoses()
            // );

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for(int j = 0; j < choreoTrajectory.getPoses().length; j++){
                poses.add(choreoTrajectory.getPoses()[j]);
            }
            //edu.wpi.first.math.trajectory.Trajectory wpilibTraj = new edu.wpi.first.math.trajectory.Trajectory();\
            //PoseTrajectory pt = new PoseTrajectory(poses);
            //poseTrajectory.concatenate(pt);
            PoseTrajectory pt = new PoseTrajectory(poses);
            trajectory = trajectory.concatenate(pt);
        }
        field2d.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("AutoTrajectory", field2d);
    }

    public void periodic(){
        startChooser.onChange((s) -> startChooserUpdate());
        reef1Chooser.onChange((s) -> reef1ChooserUpdate());
        station1Chooser.onChange((s) -> station1ChooserUpdate());
        reef2Chooser.onChange((s) -> reef2ChooserUpdate());
        station2Chooser.onChange((s) -> station2ChooserUpdate());
        reef3Chooser.onChange((s) -> getRoutine());
        SmartDashboard.putBoolean("generate preview", generatePreview);
        generatePreview = SmartDashboard.getBoolean("generate preview", false);
        if(generatePreview){
            //getRoutine();
            //System.out.println("generating");
            //updateField(starttoR1);
            //generatePreview = false;
        }
        field2d.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("AutoTrajectory", field2d);
    }

    private void instantiatePointChoosers(){
        startChooser = new SendableChooser<String>();
        startChooser.addOption("Right", "MR");
        startChooser.addOption("Mid", "MM");
        startChooser.addOption("Left", "ML");

        reef1Chooser = new SendableChooser<String>();

        station1Chooser = new SendableChooser<String>();

        reef2Chooser = new SendableChooser<String>();

        station2Chooser = new SendableChooser<String>();

        reef3Chooser = new SendableChooser<String>();

        SmartDashboard.putData("Start", startChooser);
        SmartDashboard.putData("Reef 1", reef1Chooser);
        SmartDashboard.putData("Station 1", station1Chooser);
        SmartDashboard.putData("Reef 2", reef2Chooser);
        SmartDashboard.putData("Station 2", station2Chooser);
        SmartDashboard.putData("Reef 3", reef3Chooser);
    }

    public void startChooserUpdate(){
        String prevPose = startChooser.getSelected();
        if (prevPose == null)
            return;
        reef1Chooser = new SendableChooser<String>();
        reef1Chooser.addOption("G", "RGM");
        reef1Chooser.addOption("H", "RHM");
        switch(prevPose){
            case "MR":
                reef1Chooser.addOption("E", "RER");
                reef1Chooser.addOption("F", "RFR");
            break;

            case "ML":
                reef1Chooser.addOption("I", "RIL");
                reef1Chooser.addOption("J", "RJL");
            break;

            default:
                reef1Chooser.addOption("E", "RE");
                reef1Chooser.addOption("F", "RF");
                reef1Chooser.addOption("I", "RI");
                reef1Chooser.addOption("J", "RJ");
            break;
        }
        SmartDashboard.putData("Reef 1", reef1Chooser);
    }

    public void reef1ChooserUpdate(){
        String prevPose = reef1Chooser.getSelected();
        if (prevPose == null)
            return;
        station1Chooser = new SendableChooser<String>();
        switch (prevPose.substring(2)) {
            case "L":
                station1Chooser.addOption("Back Left", "SBL");
                station1Chooser.addOption("Front Left", "SFL");
                break;

            case "R":
                station1Chooser.addOption("Back Right", "SBR");
                station1Chooser.addOption("Front Right", "SFR");
                break;
        
            default:
                station1Chooser.addOption("Back Left", "SBL");
                station1Chooser.addOption("Front Left", "SFL");
                station1Chooser.addOption("Back Right", "SBR");
                station1Chooser.addOption("Front Right", "SFR");
                break;
        }
        SmartDashboard.putData("Station 1", station1Chooser);
    }

    public void station1ChooserUpdate(){
        String prevPose = station1Chooser.getSelected();
        if (prevPose == null)
            return;
        reef2Chooser = new SendableChooser<String>();
        reef2Chooser.addOption("A", "RAM");
        reef2Chooser.addOption("B", "RBM");
        switch(prevPose.substring(2)){
            case "R":
                reef2Chooser.addOption("C", "RCR");
                reef2Chooser.addOption("D", "RDR");
            break;

            case "L":
                reef2Chooser.addOption("K", "RKL");
                reef2Chooser.addOption("L", "RLL");
            break;

            default:
                reef2Chooser.addOption("E", "RE");
                reef2Chooser.addOption("F", "RF");
                reef2Chooser.addOption("I", "RI");
                reef2Chooser.addOption("J", "RJ");
            break;
        }
        SmartDashboard.putData("Reef 2", reef2Chooser);
    }

    public void reef2ChooserUpdate(){
        String prevPose = reef2Chooser.getSelected();
        if (prevPose == null)
            return;
        station2Chooser = new SendableChooser<String>();
        switch (prevPose.substring(2)) {
            case "L":
                station2Chooser.addOption("Back Left", "SBL");
                station2Chooser.addOption("Front Left", "SFL");
                break;

            case "R":
                station2Chooser.addOption("Back Right", "SBR");
                station2Chooser.addOption("Front Right", "SFR");
                break;
        
            default:
                station2Chooser.addOption("Back Left", "SBL");
                station2Chooser.addOption("Front Left", "SFL");
                station2Chooser.addOption("Back Right", "SBR");
                station2Chooser.addOption("Front Right", "SFR");
                break;
        }
        SmartDashboard.putData("Station 2", station2Chooser);
    }
    public void station2ChooserUpdate(){
        String prevPose = station2Chooser.getSelected();
        if (prevPose == null)
            return;
        reef3Chooser = new SendableChooser<String>();
        reef3Chooser.addOption("A", "RAM");
        reef3Chooser.addOption("B", "RBM");
        switch(prevPose.substring(2)){
            case "R":
                reef3Chooser.addOption("C", "RCR");
                reef3Chooser.addOption("D", "RDR");
            break;

            case "L":
                reef3Chooser.addOption("K", "RKL");
                reef3Chooser.addOption("L", "RLL");
            break;

            default:
                reef3Chooser.addOption("E", "RE");
                reef3Chooser.addOption("F", "RF");
                reef3Chooser.addOption("I", "RI");
                reef3Chooser.addOption("J", "RJ");
            break;
        }
        SmartDashboard.putData("Reef 3", reef3Chooser);
    }
}
