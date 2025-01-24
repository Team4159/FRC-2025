package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private SparkMax roller;
    private SparkMax pivot;

    public AlgaeIntake() {
        roller = new SparkMax(Constants.AlgaeIntake.rollerID, MotorType.kBrushless);
        pivot = new SparkMax(Constants.AlgaeIntake.pivotID, MotorType.kBrushless);
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public void stopRoller() {
        roller.set(0);
    }
    /**@param angle radians*/
    public void setIntakeAngle(double angle) {
        pivot.getClosedLoopController().setReference(Units.radiansToRotations(angle), ControlType.kPosition);
    }

    public class ChangeState extends Command {
        private final Constants.AlgaeIntake.AlgaeIntakeState desiredState;

        public ChangeState(Constants.AlgaeIntake.AlgaeIntakeState desiredState) {
            this.desiredState = desiredState;
            addRequirements(AlgaeIntake.this);
        }

        @Override
        public void initialize() {
            setRollerSpeed(desiredState.speed);
            setIntakeAngle(desiredState.angle);
        }

        @Override
        public void end(boolean interrupted) {
            stopRoller();
            setIntakeAngle(Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle);
        }
    }
}
