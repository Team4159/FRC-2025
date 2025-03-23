package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralManipulator.PivotState;
import frc.robot.Constants.CoralManipulator.RollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class AutoOuttake extends SequentialCommandGroup {
    public AutoOuttake(CoralManipulator coralManip, Elevator elev, boolean trough) {
        addCommands(
                coralManip.new ChangeState(null, trough ? RollerState.OUTTAKETROUGH : RollerState.OUTTAKE)
                        .withTimeout(2).finallyDo((interrupted) -> AutoOuttake.this
                                .andThen(
                                new ParallelCommandGroup(
                                    coralManip.new ChangeState(PivotState.INTAKE, RollerState.INTAKE),
                                    elev.new ChangeState(ElevatorState.INTAKE, true)
                                )    
                                )));
    }
}