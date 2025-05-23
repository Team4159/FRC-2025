package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class AutoOuttake extends Command{
    private CoralManipulator coralManipulator;
    private Elevator elevator;
    private boolean backupTimer, trough, usingTimer;
    private double backupTimeOffset;
    private double timeOffset;
    private Timer timer;

    /** @param backupTimer will end the command automatically after a certain amout of time. used for auto*/
    public AutoOuttake(CoralManipulator coralManipulator, Elevator elevator, boolean backupTimer, boolean trough){
        this.coralManipulator = coralManipulator;
        this.elevator = elevator;
        this.backupTimer = backupTimer;
        this.trough = trough;
        timer = new Timer();
        usingTimer = false;
        addRequirements(coralManipulator, elevator);
    }

    /** @param backupTimer will end the command automatically after a certain amout of time. used for auto*/
    public AutoOuttake(CoralManipulator coralManipulator, Elevator elevator, boolean backupTimer){
        this(coralManipulator, elevator, backupTimer, false);
    }

    /** ends automatically when there is no coral detected.*/
    public AutoOuttake(CoralManipulator coralManipulator, Elevator elevator){
        this(coralManipulator, elevator, false, false);
    }

    @Override
    public void initialize(){
        if(trough){
            coralManipulator.setRollerGoalState(CoralManipulatorRollerState.OUTTAKETROUGH);
        }
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.OUTTAKE);
        backupTimeOffset = Timer.getFPGATimestamp();
        usingTimer = false;
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished(){
        if(!coralManipulator.hasCoral()){
            timer.start();
            usingTimer = true;
        }
        System.out.println(usingTimer);
        return (!coralManipulator.hasCoral() && usingTimer && timer.get() > 1)|| 
               (backupTimer && Timer.getFPGATimestamp() - backupTimeOffset > 2);
    }

    @Override
    public void end(boolean interrupted){
        timer.stop();
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
        if(!interrupted){
            coralManipulator.setPivotGoalState(CoralManipulatorPivotState.INTAKE);
            //elevator.setGoalState(ElevatorState.INTAKE);
        }
    }
}
