package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntake.AlgaeIntakeState;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex roller; //vortex
    private SparkMax pivot; //neo
    private double targetAngle;
    private DutyCycleEncoder encoder;

    public AlgaeIntake() {
        roller = new SparkFlex(Constants.AlgaeIntake.rollerID, MotorType.kBrushless);
        pivot = new SparkMax(Constants.AlgaeIntake.pivotID, MotorType.kBrushless);
        targetAngle = Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle;
        encoder = new DutyCycleEncoder(0);
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public void stopRoller() {
        roller.stopMotor();
    }
    /**@param angle radians*/
    public void setIntakeAngle(double angle) {
        targetAngle = angle;
        Constants.AlgaeIntake.pidController.reset();
    }

    public boolean isAtSetpoint(){
        return Math.abs(encoder.get() - targetAngle) < Constants.AlgaeIntake.tolerance;
    }

    public boolean isStowed(){
        return targetAngle == AlgaeIntakeState.STOW.angle && isAtSetpoint();
    }

    @Override
    public void periodic(){
        //double currentAngle = pivot.getAbsoluteEncoder().getPosition() * Math.PI;
        double currentAngle = encoder.get();
        if(currentAngle < 0.25){
            currentAngle = 1;
        }
        SmartDashboard.putNumber("AlgaeManip angle", currentAngle);
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
        public boolean isFinished(){
            return isAtSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            stopRoller();
            if (stow) setIntakeAngle(Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle);
        }
    }
}
