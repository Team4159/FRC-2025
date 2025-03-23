package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulator.PivotState;
import frc.robot.Constants.CoralManipulator.RollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;

public class AutoIntake extends Command{
    private CoralManipulator coralManipulator;
    private Elevator elevator;
    private double timeOffset;
    private boolean useBeamBreak;
    //private double simTimeOffset;
    private LED led;

    /** @param useBeamBreak if set to true the command will end when the beam break detects a coral */
    public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led, boolean useBeamBreak){
        this.coralManipulator = coralManipulator;
        this.elevator = elevator;
        this.useBeamBreak = useBeamBreak;
        this.led = led;
        //led.setDefaultCommand(led.new ChromaLED((double i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly());
        addRequirements(coralManipulator, elevator);
    }

    /** the command will end when the beam break detects coral */
    public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led){
        this(coralManipulator, elevator, led, true);
    }

    @Override
    public void initialize(){
        coralManipulator.setGoalState(PivotState.INTAKE, RollerState.INTAKE);
        elevator.setGoalState(ElevatorState.INTAKE);
        // simTimeOffset = Timer.getFPGATimestamp();
        led.rainbow();
    }

    @Override
    public boolean isFinished(){
        //if(RobotBase.isSimulation() && Timer.getFPGATimestamp() - simTimeOffset > 5)
            //return true;
        if(coralManipulator.hasCoral() && timeOffset == 0){
            timeOffset = Timer.getFPGATimestamp();
        }
        return useBeamBreak && coralManipulator.hasCoral() && Timer.getFPGATimestamp() - timeOffset > 0;
    }

    @Override
    public void end(boolean interrupted){
        coralManipulator.setGoalState(PivotState.STOW ,RollerState.PASSIVE);
    }
}
