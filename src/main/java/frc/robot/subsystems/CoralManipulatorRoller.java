package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulatorRoller extends SubsystemBase{
    public SparkMax rollerMotor;
    public DigitalInput beamBreak;

    public CoralManipulatorRoller(){
        rollerMotor = new SparkMax(Constants.CoralManipulator.rollerMotorID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.CoralManipulator.beamBreakDIO);
    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public void setGoalSpin(double spin){
        rollerMotor.set(spin);
    }

    public class ChangeState extends Command{
        
    }
}
