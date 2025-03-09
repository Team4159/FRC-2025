package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulatorPivot;
import frc.robot.subsystems.CoralManipulatorRoller;
import frc.robot.subsystems.Elevator;

public class AutoIntake extends Command{
    private CoralManipulatorPivot coralManipulatorPivot;
    private CoralManipulatorRoller coralManipulatorRoller;
    private Elevator elevator;
    private double timeOffset;
    private boolean useBeamBreak;
    private double simTimeOffset;

    /** @param useBeamBreak if set to true the command will end when the beam break detects a coral */
    public AutoIntake(CoralManipulatorPivot coralManipulatorPivot, CoralManipulatorRoller coralManipulatorRoller, Elevator elevator, boolean useBeamBreak){
        this.coralManipulatorPivot = coralManipulatorPivot;
        this.coralManipulatorRoller = coralManipulatorRoller;
        this.elevator = elevator;
        this.useBeamBreak = useBeamBreak;
        addRequirements(coralManipulatorPivot, coralManipulatorRoller, elevator);
    }

    /** the command will end when the beam break detects coral */
    public AutoIntake(CoralManipulatorPivot coralManipulatorPivot, CoralManipulatorRoller coralManipulatorRoller, Elevator elevator){
        this(coralManipulatorPivot, coralManipulatorRoller, elevator, true);
    }

    @Override
    public void initialize(){
        coralManipulatorPivot.setGoalState(CoralManipulatorPivotState.INTAKE);
        //coralManipulatorRoller.setGoalState(CoralManipulatorRollerState.INTAKE);
        elevator.setGoalState(ElevatorState.INTAKE);
        simTimeOffset = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished(){
        System.out.println(Timer.getFPGATimestamp() - simTimeOffset);
        if(RobotBase.isSimulation() && Timer.getFPGATimestamp() - simTimeOffset > 5)
            return true;
        if(coralManipulatorRoller.hasCoral() && timeOffset == 0){
            timeOffset = Timer.getFPGATimestamp();
        }
        return useBeamBreak && coralManipulatorRoller.hasCoral() && Timer.getFPGATimestamp() - timeOffset > 0;
    }

    @Override
    public void end(boolean interrupted){
        coralManipulatorRoller.setGoalState(CoralManipulatorRollerState.PASSIVE);
        //coralManipulatorPivot.setGoalState(CoralManipulatorPivotState.L2AND3);
    }
}
