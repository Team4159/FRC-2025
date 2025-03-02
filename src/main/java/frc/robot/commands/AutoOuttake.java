package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulatorPivot;
import frc.robot.subsystems.CoralManipulatorRoller;
import frc.robot.subsystems.Elevator;

public class AutoOuttake extends Command{
    private CoralManipulatorPivot coralManipulatorPivot;
    private CoralManipulatorRoller coralManipulatorRoller;
    private Elevator elevator;
    private boolean backupTimer;
    private double backupTimeOffset;
    private double timeOffset;

    /** @param backupTimer will end the command automatically after a certain amout of time. used for auto*/
    public AutoOuttake(CoralManipulatorPivot coralManipulatorPivot, CoralManipulatorRoller coralManipulatorRoller, Elevator elevator, boolean backupTimer){
        this.coralManipulatorPivot = coralManipulatorPivot;
        this.coralManipulatorRoller = coralManipulatorRoller;
        this.elevator = elevator;
        this.backupTimer =  backupTimer;
        addRequirements(coralManipulatorPivot, coralManipulatorRoller, elevator);
    }

    /** ends automatically when there is no coral detected.*/
    public AutoOuttake(CoralManipulatorPivot coralManipulatorPivot, CoralManipulatorRoller coralManipulatorRoller, Elevator elevator){
        this(coralManipulatorPivot, coralManipulatorRoller, elevator, false);
    }

    @Override
    public void initialize(){
        coralManipulatorRoller.setGoalState(CoralManipulatorRollerState.OUTTAKE);
        backupTimeOffset = Timer.getFPGATimestamp();
        // if(coralManipulatorPivot.isL4()){
        //     coralManipulatorPivot.setGoalState(CoralManipulatorPivotState.L4FINAL);
        // }
    }

    @Override
    public boolean isFinished(){
        System.out.println("using autooutake");
        if(!coralManipulatorRoller.hasCoral() && timeOffset == 0){
            timeOffset = Timer.getFPGATimestamp();
        }
        return (!coralManipulatorRoller.hasCoral() && Timer.getFPGATimestamp() - timeOffset > 1.5 )|| 
               (backupTimer && Timer.getFPGATimestamp() - backupTimeOffset > 2);
        //return false;
    }

    @Override
    public void end(boolean interrupted){
        coralManipulatorRoller.setGoalState(CoralManipulatorRollerState.OFF);
        if(!interrupted){
            //coralManipulatorPivot.setGoalState(CoralManipulatorPivotState.INTAKE);
            //elevator.setGoalState(ElevatorState.INTAKE);
        }
    }
}
