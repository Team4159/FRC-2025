package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;

public class AutoIntake extends Command{
    private CoralManipulator coralManipulator;
    private Elevator elevator;
    private double timeOffset;
    private Timer timer;
    private boolean useBeamBreak;
    private LED led;

    /** @param useBeamBreak if set to true the command will end when the beam break detects a coral */
    public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led, boolean useBeamBreak){
        this.coralManipulator = coralManipulator;
        this.elevator = elevator;
        this.useBeamBreak = useBeamBreak;
        this.led = led;
        timer = new Timer();
        addRequirements(coralManipulator, elevator);
    }

    /** the command will end when the beam break detects coral */
    public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led){
        this(coralManipulator, elevator, led, true);
    }

    @Override
    public void initialize(){
        coralManipulator.setPivotGoalState(CoralManipulatorPivotState.INTAKE);
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.INTAKE);
        elevator.setGoalState(ElevatorState.INTAKE);
        led.rainbow();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished(){
        if(RobotBase.isSimulation() && timer.get() > 0.75) return true; 
        if(coralManipulator.hasCoral() && timeOffset == 0){
            timeOffset = Timer.getFPGATimestamp();
        }
        return useBeamBreak && coralManipulator.hasCoral() && Timer.getFPGATimestamp() - timeOffset > 0;
    }

    @Override
    public void end(boolean interrupted){
        coralManipulator.setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
        //coralManipulator.setPivotGoalState(CoralManipulatorPivotState.L2AND3);
    }
}
