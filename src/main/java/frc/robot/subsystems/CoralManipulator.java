package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.PivotState;
import frc.robot.Constants.CoralManipulator.RollerState;

public class CoralManipulator extends SubsystemBase{
    private final SparkBase angle, roller;
    private final DigitalInput beamBreak;

    private PivotState targetPivotState;
    private RollerState targetRollerState;

    private final ProfiledPIDController anglePID = new ProfiledPIDController(
        Constants.CoralManipulator.kP, Constants.CoralManipulator.kI, Constants.CoralManipulator.kD, Constants.CoralManipulator.constraints) {{
            setTolerance(Constants.CoralManipulator.angleTolerance);
        }};
    private final ArmFeedforward angleFFEmpty = new ArmFeedforward(
        Constants.CoralManipulator.kS,
        Constants.CoralManipulator.kGEmpty,
        Constants.CoralManipulator.kV,
        Constants.CoralManipulator.kA
    );
    private final ArmFeedforward angleFFFull = new ArmFeedforward(
        Constants.CoralManipulator.kS,
        Constants.CoralManipulator.kGFull,
        Constants.CoralManipulator.kV,
        Constants.CoralManipulator.kA
    );

    public CoralManipulator() {
        angle  = new SparkFlex(Constants.CoralManipulator.angleMotorID, MotorType.kBrushless);
        roller = new SparkMax(Constants.CoralManipulator.rollerMotorID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.CoralManipulator.beamBreakDIO);
    }

    @Override
    public void periodic() {
        angle.setVoltage(
            anglePID.calculate(getAngularPosition(), targetPivotState.angle) + 
            (hasCoral() ? angleFFFull : angleFFEmpty).calculateWithVelocities(getAngularPosition(), getAngluarVelocity(), 0d)
        );
        roller.set(targetRollerState.spin);
    }

    /**
     * dont use this please, its on there as a drop-in replace for auto code
     */
    public void setGoalState(PivotState p, RollerState r) {
        targetPivotState = p; targetRollerState = r;
    }
    
    public boolean hasCoral(){
        return !beamBreak.get();
    }

    /**
     * @return radians
     */
    private double getAngularPosition() {
        return Units.rotationsToRadians(angle.getAbsoluteEncoder().getPosition()) + Constants.CoralManipulator.FFOffset;
    }

    /**
     * @return radians/second
     */
    private double getAngluarVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(angle.getEncoder().getVelocity());
    }

    public class ChangeState extends Command {
        private final PivotState pivot;
        private final RollerState roller;

        private final boolean stowAfter;
        private boolean expectBeamBreak; // true if we need to wait for the beam break to proc
        private final Timer coralTransitTimer = new Timer();

        private ChangeState(PivotState pivot, RollerState roller, boolean stowAfter) {
            this.pivot = pivot;
            this.roller = roller;
            this.stowAfter = stowAfter;
            addRequirements(CoralManipulator.this);
        }

        public ChangeState(PivotState pivot) {
            this(pivot, null, false);
        }

        public ChangeState(PivotState pivot, RollerState roller) {
            this(pivot, roller, true);
        }

        @Override
        public void initialize() {
            targetPivotState = pivot;
            expectBeamBreak = targetRollerState == RollerState.INTAKE && !hasCoral();
        }

        @Override
        public void execute() {
            if (roller != null && anglePID.atGoal() && targetRollerState != roller) {
                targetRollerState = roller;
                coralTransitTimer.restart();
            }

            SmartDashboard.putNumber("armsetpoint", Units.radiansToDegrees(pivot.angle));
            SmartDashboard.putNumber("armposition", Units.radiansToDegrees(getAngularPosition()));
        }

        @Override
        public boolean isFinished() {
            if (roller == null) return anglePID.atGoal(); // if it's only preparing
            if (expectBeamBreak) return anglePID.atGoal() && hasCoral(); // if it's intaking
            return coralTransitTimer.hasElapsed(Constants.CoralManipulator.coralTransitTime); // if it's outtaking, generally
        }

        @Override
        public void end(boolean interrupted) {
            if (stowAfter) targetPivotState = PivotState.STOW;
            targetRollerState = interrupted || hasCoral() ? RollerState.PASSIVE : RollerState.IDLE;
        }
    }
}
