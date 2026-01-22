package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// WARNING CODE IS MODIFIED FOR PROTOTYPE TESTING
// CHANGE SETPOINTS BEFORE RUNNING

public class AlgaeIntake extends SubsystemBase {
    private TalonFX roller; //x44
    private SparkFlex pivot; //vortex
    private double targetAngle;
    private RelativeEncoder encoder;

    public AlgaeIntake() {
        roller = new TalonFX(Constants.AlgaeIntake.rollerID);
        pivot = new SparkFlex(Constants.AlgaeIntake.pivotID, MotorType.kBrushless);
        targetAngle = Constants.AlgaeIntake.AlgaeIntakeState.INTAKE.angle;
        encoder = pivot.getEncoder();
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

    @Override
    public void periodic(){
        //double currentAngle = pivot.getAbsoluteEncoder().getPosition() * Math.PI;
        double currentAngle = encoder.getPosition(); 
        // if(currentAngle < 0.25) {
        //     currentAngle = 1;
        // }
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
        public void end(boolean interrupted) {
            stopRoller();
            if (stow) setIntakeAngle(Constants.AlgaeIntake.AlgaeIntakeState.STOW.angle);
        }
    }
}
