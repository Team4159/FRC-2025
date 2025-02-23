package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;

public class CoralManipulatorRoller extends SubsystemBase{
    public SparkMax rollerMotor;
    public DigitalInput beamBreak;

    public CoralManipulatorRoller(){
        rollerMotor = new SparkMax(Constants.CoralManipulator.rollerMotorID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.CoralManipulator.beamBreakDIO);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Coral", hasCoral());
    }

    public boolean hasCoral(){
        return !beamBreak.get();
    }

    /** @param desiredState the desired CoralManipulatorRollerState*/
    public void setGoalState(CoralManipulatorRollerState desiredState){
        rollerMotor.set(desiredState.spin);
    }

    public class ChangeState extends Command{
        private CoralManipulatorRollerState state;

        public ChangeState(CoralManipulatorRollerState state){
            this.state = state;
            addRequirements(CoralManipulatorRoller.this);
        }

        @Override
        public void initialize(){
            CoralManipulatorRoller.this.setGoalState(state);
        }

        @Override
        public void end(boolean interrupted){
            CoralManipulatorRoller.this.setGoalState(CoralManipulatorRollerState.OFF);
        }
    }
}
