package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.subsystems.CoralManipulator;

public class AutoIntakeRoller extends Command{
    private CoralManipulator coralManipulator;
    private double intaketime;

    public AutoIntakeRoller(CoralManipulator coralManipulator){
        this.coralManipulator = coralManipulator;
        intaketime = 0;
    }

    @Override
    public void initialize(){
        coralManipulator.setPivotGoalState(CoralManipulatorPivotState.INTAKE);
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.INTAKE);
    }

    @Override
    public boolean isFinished(){
        if(coralManipulator.hasCoral() && intaketime == 0) intaketime = Timer.getFPGATimestamp();
        return coralManipulator.hasCoral() && Timer.getFPGATimestamp() - intaketime > 0;
    }

    @Override
    public void end(boolean interrupted){
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
        //coralManipulator.setPivotGoalState(CoralManipulatorPivotState.L2AND3);
    }
}
