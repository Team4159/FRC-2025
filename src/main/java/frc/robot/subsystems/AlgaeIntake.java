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
    private double targetAngle;

    public AlgaeIntake() {
        roller = new SparkMax(Constants.AlgaeIntake.rollerID, MotorType.kBrushless);
        pivot = new SparkMax(Constants.AlgaeIntake.pivotID, MotorType.kBrushless);
        targetAngle = Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle;
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public void stopRoller() {
        roller.set(0);
    }
    /**@param angle radians*/
    public void setIntakeAngle(double angle) {
        targetAngle = angle;
        Constants.AlgaeIntake.pidController.reset();
    }

    @Override
    public void periodic(){
        double currentAngle = pivot.getAbsoluteEncoder().getPosition() * Math.PI;
        //System.out.println(targetAngle + " " + currentAngle);
        pivot.set(-Constants.AlgaeIntake.pidController.calculate(currentAngle, targetAngle));
    }

    public class ChangeState extends Command {
        private final Constants.AlgaeIntake.AlgaeIntakeState desiredState;

        private final boolean stow;

        public ChangeState(Constants.AlgaeIntake.AlgaeIntakeState desiredState, boolean stow) {
            this.desiredState = desiredState;
            this.stow = stow;
            addRequirements(AlgaeIntake.this);
        }

        public ChangeState(Constants.AlgaeIntake.AlgaeIntakeState desiredState) {
            this(desiredState, false); 
        }

        @Override
        public void initialize() {
            setRollerSpeed(desiredState.speed);
            setIntakeAngle(desiredState.angle);
        }

        @Override
        public void end(boolean interrupted) {
            stopRoller();
            if (stow) setIntakeAngle(Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle);
        }
    }
}
