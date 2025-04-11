package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deepclimb extends SubsystemBase{
  SparkFlex motor;
  
  public Deepclimb(){
    motor = new SparkFlex(Constants.Deepclimb.deviceID, MotorType.kBrushless);
  }
  public void spin(double speed){
    motor.set(speed);

  }
  public void liftArm(double angle){
    motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void pivot(double speed){
    
  }

  public class ChangeState extends Command {
    private final Constants.Deepclimb.deepClimbStates selectedState;
    public ChangeState(Constants.Deepclimb.deepClimbStates state){
      selectedState = state;
      addRequirements(Deepclimb.this); // use subsystem self as part of command
    }
  

    @Override
    public void initialize(){
      spin(selectedState.speed);
    }

    @Override
    public void end(boolean interrupted){
      motor.set(0);
    }
  }
}