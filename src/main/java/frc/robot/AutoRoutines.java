package frc.robot;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoOuttake;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines extends SubsystemBase{
    private final AutoFactory m_factory;
    private final Elevator elevator;
    private final CoralManipulator coralManipulator;
    private final CommandSwerveDrivetrain swerve;
    private final LED led;
    private SendableChooser<String> startChooser, reef1Chooser, station1Chooser, reef2Chooser;//, station2Chooser, reef3Chooser;
    private SendableChooser<Integer> typeChooser;
    private Field2d field2d;
    private AutoRoutine routine;

    // private AutoRoutine routine;
    // private AutoTrajectory starttoR1;
    // private AutoTrajectory R1toS1;
    // private AutoTrajectory S1toR2;
    // private AutoTrajectory R2toS2;
    // private AutoTrajectory S2toR3;

    private boolean generatePreview;

    private edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain swerve, Elevator elevator, CoralManipulator coralManipulator, LED led) {
        m_factory = factory;
        this.elevator = elevator;
        this.coralManipulator = coralManipulator;
        this.swerve = swerve;
        this.led = led;
        field2d = new Field2d();
        instantiatePointChoosers();
    }

    /** 
     * Does not generate a preview.
     * @return The Choreo AutoRoutine with the desired criteria from SmartDashboard.
     */
    public AutoRoutine getRoutine(){
        return getRoutine(false);
    }

    /**
     * @param preview If true the function will also generate a preview on a Field2d for SmartDashboard.
     * @return The Choreo AutoRoutine with the desired criteria from SmartDashboard.
     */
    public AutoRoutine getRoutine(boolean preview){
        final AutoRoutine routine = m_factory.newRoutine("routine");
        final String start = startChooser.getSelected();
        final String s1 = station1Chooser.getSelected();
        //final String s2 = station2Chooser.getSelected();
        if(start == null || reef1Chooser.getSelected() == null || s1 == null || reef2Chooser.getSelected() == null){
            System.out.println("null values");
            return routine;
        }

        final String r1 = reef1Chooser.getSelected().substring(0, 2);
        final String r2 = reef2Chooser.getSelected().substring(0, 2);
        //final String r3 = reef3Chooser.getSelected().substring(0, 2);

        final AutoTrajectory starttoR1 = routine.trajectory(start + "to" + r1);
        final AutoTrajectory R1toS1 = routine.trajectory(r1 + "to" + s1);
        //final AutoTrajectory S1toTrough = routine.trajectory(s1 + "toTrough");
        final AutoTrajectory S1toR2 = routine.trajectory(s1 + "to" + r2);
        // final AutoTrajectory R2toS2 = routine.trajectory(r2 + "to" + s2);
        // final AutoTrajectory S2toR3 = routine.trajectory(s2 + "to" + r3);
        //a type of 2 corresponds to a 2 coral auto
        if(typeChooser.getSelected() == 2){
            routine.active().onTrue(
                starttoR1.resetOdometry()
                //go to first L4
                .andThen(
                    new ParallelCommandGroup(
                        coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4),
                        elevator.new ChangeState(ElevatorState.L4),
                        new SequentialCommandGroup(starttoR1.cmd(), new InstantCommand(() -> swerve.stopSwerve()))))
                //align
                //.andThen(new AutoAlign(swerve, led, isleft1))
                //outtake
                .andThen(new AutoOuttake(coralManipulator, elevator, true))
                //go to first station
                .andThen(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(R1toS1.cmd(), new InstantCommand(() -> swerve.stopSwerve())),
                        new SequentialCommandGroup(new WaitCommand(0.25), elevator.new ChangeState(ElevatorState.INTAKE))))
                //intake
                .andThen(new AutoIntake(coralManipulator, elevator, led))
                //go to second l4
                .andThen(
                    new ParallelCommandGroup(
                        coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4),
                        elevator.new ChangeState(ElevatorState.L4),
                        new SequentialCommandGroup(S1toR2.cmd(), new InstantCommand(() -> swerve.stopSwerve()))))
                //align
                //.andThen(new AutoAlign(swerve, led, isleft2))
                //outtake
                .andThen(new AutoOuttake(coralManipulator, elevator, true))
                .andThen(new SequentialCommandGroup(new WaitCommand(0.25), elevator.new ChangeState(ElevatorState.INTAKE)))
                //3 coral probably never to be used :(
                // .andThen(new SequentialCommandGroup(R2toS2.cmd(), new InstantCommand(() -> swerve.stopSwerve())))
                // .andThen(new AutoIntake(coralManipulatorPivot, coralManipulatorRoller, elevator, led))
                // .andThen(
                //     new ParallelCommandGroup(
                //         //coralManipulatorPivot.new ChangeState(CoralManipulatorPivotState.L4, false),
                //         //elevator.new ChangeState(ElevatorState.L4, false),
                //         new SequentialCommandGroup(S2toR3.cmd(), new InstantCommand(() -> swerve.stopSwerve()))))
                // .andThen(new InstantCommand(() -> swerve.stopSwerve()))
                // .andThen(new AutoOuttake(coralManipulatorPivot, coralManipulatorRoller, elevator, true))
            );
            if(preview){
                //update smartdashboard preview
                updateField(starttoR1, R1toS1, S1toR2);
            }
        }
        else if(typeChooser.getSelected() == 1){
            routine.active().onTrue(
                starttoR1.resetOdometry()
                //go to first L4
                .andThen(
                    new ParallelCommandGroup(
                        coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4),
                        elevator.new ChangeState(ElevatorState.L4),
                        new SequentialCommandGroup(starttoR1.cmd(), new InstantCommand(() -> swerve.stopSwerve()))))
                //align
                //.andThen(new AutoAlign(swerve, led, isleft1))
                //outtake
                .andThen(new AutoOuttake(coralManipulator, elevator, false))
                .andThen(new SequentialCommandGroup(new WaitCommand(0.25), elevator.new ChangeState(ElevatorState.INTAKE)))
            );
            if(preview){
                updateField(starttoR1);
            }
        }
        //by default a trough auto is created
        else {
            final AutoTrajectory starttoTrough = routine.trajectory(startChooser.getSelected() + "Trough");
            routine.active().onTrue(
                starttoTrough.resetOdometry()
                //go to trough location
                .andThen(
                    new ParallelCommandGroup(
                        coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L2),
                        elevator.new ChangeState(ElevatorState.L2),
                        new SequentialCommandGroup(starttoTrough.cmd(), new InstantCommand(() -> swerve.stopSwerve()))))
                //outtake at trough speed to mitigate bouncing out
                .andThen(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKETROUGH))
                .andThen(new SequentialCommandGroup(new WaitCommand(0.25), elevator.new ChangeState(ElevatorState.INTAKE)))
            );
            if(preview){
                updateField(starttoTrough);
            }
        }
        this.routine = routine;
        return routine;
    }

    /**
     * @return The current Choreo AutoRoutine(Using criteria from SmartDashboard) if generated(Driver has to press "generate" button on dashboard), otherwise it runs the getRoutine function and returns the result.
     */
    public AutoRoutine getGeneratedRoutine(){
        if(routine == null){
            return getRoutine();
        }
        return routine;
    }

    /**
     * displays an autoroutine on smartdashboard
     * @param trajectories the Choreo AutoTrajectories that make up the routine desired to be displayed
     */
    public void updateField(AutoTrajectory... trajectories){
        trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        for(int i = 0; i < trajectories.length; i++){
            Trajectory<SwerveSample> choreoTrajectory = trajectories[i].getRawTrajectory();

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for(int j = 0; j < choreoTrajectory.getPoses().length; j++){
                poses.add(choreoTrajectory.getPoses()[j]);
            }
            PoseTrajectory pt = new PoseTrajectory(poses);
            trajectory = trajectory.concatenate(pt);
        }
        field2d.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("AutoTrajectory", field2d);
    }

    @Override
    public void periodic(){
        //check for updates to the SendableChoosers
        startChooser.onChange((s) -> startChooserUpdate());
        reef1Chooser.onChange((s) -> reef1ChooserUpdate());
        station1Chooser.onChange((s) -> station1ChooserUpdate());
        //reef2Chooser.onChange((s) -> reef2ChooserUpdate());
        //station2Chooser.onChange((s) -> station2ChooserUpdate());
        //check for updates to the generate button
        generatePreview = SmartDashboard.getBoolean("generate", false);
        if(generatePreview){
            getRoutine(true);
            generatePreview = false;
            SmartDashboard.putBoolean("generate", generatePreview);
        }
        field2d.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("AutoTrajectory", field2d);
    }

    private void instantiatePointChoosers(){
        SmartDashboard.putBoolean("generate", generatePreview);
        typeChooser = new SendableChooser<Integer>();
        typeChooser.addOption("Trough", 0);
        typeChooser.addOption("1CL4", 1);
        //typeChooser.addOption("3CL4", 3);
        typeChooser.setDefaultOption("2CL4", 2);
        startChooser = new SendableChooser<String>();
        startChooser.addOption("Right", "MR");
        startChooser.addOption("Mid", "MM");
        startChooser.addOption("Left", "ML");

        reef1Chooser = new SendableChooser<String>();

        station1Chooser = new SendableChooser<String>();

        reef2Chooser = new SendableChooser<String>();

        //station2Chooser = new SendableChooser<String>();

        //reef3Chooser = new SendableChooser<String>();

        SmartDashboard.putData("Auto Type", typeChooser);
        SmartDashboard.putData("Start", startChooser);
        SmartDashboard.putData("Reef 1", reef1Chooser);
        SmartDashboard.putData("Station 1", station1Chooser);
        SmartDashboard.putData("Reef 2", reef2Chooser);
        // SmartDashboard.putData("Station 2", station2Chooser);
        // SmartDashboard.putData("Reef 3", reef3Chooser);
    }

    public void startChooserUpdate(){
        String prevPose = startChooser.getSelected();
        if (prevPose == null)
            return;
        reef1Chooser = new SendableChooser<String>();
        reef1Chooser.addOption("G", "RGR");
        reef1Chooser.addOption("H", "RHL");
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
                reef1Chooser.addOption("E", "RER");
                reef1Chooser.addOption("F", "RFR");
                reef1Chooser.addOption("I", "RIL");
                reef1Chooser.addOption("J", "RJL");
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
        }
        SmartDashboard.putData("Reef 2", reef2Chooser);
    }

    // public void reef2ChooserUpdate(){
    //     String prevPose = reef2Chooser.getSelected();
    //     if (prevPose == null)
    //         return;
    //     station2Chooser = new SendableChooser<String>();
    //     switch (prevPose.substring(2)) {
    //         case "L":
    //             station2Chooser.addOption("Back Left", "SBL");
    //             station2Chooser.addOption("Front Left", "SFL");
    //         break;

    //         case "R":
    //             station2Chooser.addOption("Back Right", "SBR");
    //             station2Chooser.addOption("Front Right", "SFR");
    //         break;
        
    //         default:
    //             station2Chooser.addOption("Back Left", "SBL");
    //             station2Chooser.addOption("Front Left", "SFL");
    //             station2Chooser.addOption("Back Right", "SBR");
    //             station2Chooser.addOption("Front Right", "SFR");
    //         break;
    //     }
    //     SmartDashboard.putData("Station 2", station2Chooser);
    // }
    // public void station2ChooserUpdate(){
    //     String prevPose = station2Chooser.getSelected();
    //     if (prevPose == null)
    //         return;
    //     reef3Chooser = new SendableChooser<String>();
    //     reef3Chooser.addOption("A", "RAM");
    //     reef3Chooser.addOption("B", "RBM");
    //     switch(prevPose.substring(2)){
    //         case "R":
    //             reef3Chooser.addOption("C", "RCR");
    //             reef3Chooser.addOption("D", "RDR");
    //         break;

    //         case "L":
    //             reef3Chooser.addOption("K", "RKL");
    //             reef3Chooser.addOption("L", "RLL");
    //         break;
    //     }
    //     SmartDashboard.putData("Reef 3", reef3Chooser);
    // }
}
