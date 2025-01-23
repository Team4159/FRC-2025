package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
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
}
