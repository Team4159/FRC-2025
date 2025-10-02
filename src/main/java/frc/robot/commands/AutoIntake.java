package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.Constants.AlgaeIntake.AlgaeIntakeState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;

// public class AutoIntake extends Command{
//     private CoralManipulator coralManipulator;
//     private Elevator elevator;
//     private double timeOffset;
//     private boolean useBeamBreak;
//     private LED led;

//     /** @param useBeamBreak if set to true the command will end when the beam break detects a coral */
//     public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led, boolean useBeamBreak){
//         this.coralManipulator = coralManipulator;
//         this.elevator = elevator;
//         this.useBeamBreak = useBeamBreak;
//         this.led = led;
//         addRequirements(coralManipulator, elevator);
//     }

//     /** the command will end when the beam break detects coral */
//     public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, LED led){
//         this(coralManipulator, elevator, led, true);
//     }

//     @Override
//     public void initialize(){
//         coralManipulator.setPivotGoalState(CoralManipulatorPivotState.INTAKE);
//         coralManipulator.setRollerGoalState(CoralManipulatorRollerState.INTAKE);
//         elevator.setGoalState(ElevatorState.INTAKE);
//         led.rainbow();
//     }

//     @Override
//     public boolean isFinished(){
//         if(coralManipulator.hasCoral() && timeOffset == 0){
//             timeOffset = Timer.getFPGATimestamp();
//         }
//         return useBeamBreak && coralManipulator.hasCoral() && Timer.getFPGATimestamp() - timeOffset > 0;
//     }

//     @Override
//     public void end(boolean interrupted){
//         coralManipulator.setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
//         //coralManipulator.setPivotGoalState(CoralManipulatorPivotState.L2AND3);
//     }
// }

public class AutoIntake extends ConditionalCommand{
    private CoralManipulator coralManipulator;
    private Elevator elevator;
    private double timeOffset;
    private LED led;
    private AlgaeIntake alageIntake;

    public AutoIntake(CoralManipulator coralManipulator, Elevator elevator, AlgaeIntake algaeIntake, LED led){
        super(//if the algae is stowed, intake normally
            new ParallelCommandGroup(
                new InstantCommand(() -> led.rainbow()),
                elevator.new ChangeState(ElevatorState.INTAKE),
                coralManipulator.new ChangePivotState(CoralManipulatorPivotState.INTAKE),
                new AutoIntakeRoller(coralManipulator)),
            //otherwise coral ALGAEDEPLOY and elevator INTAKE(in parallel) -> algae STOW -> coral STOW
            new SequentialCommandGroup(
                new InstantCommand(() -> led.rainbow()),
                new ParallelCommandGroup(
                    elevator.new ChangeState(ElevatorState.INTAKE),
                    coralManipulator.new ChangeState(CoralManipulatorPivotState.ALGAEDEPLOY, CoralManipulatorRollerState.INTAKE)),
                algaeIntake.new ChangeState(AlgaeIntakeState.STOW),
                new AutoIntakeRoller(coralManipulator)),
            //check if the algaemanip is stowed
            algaeIntake::isStowed);
    }
}
