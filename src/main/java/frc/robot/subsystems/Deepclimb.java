package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deepclimb extends SubsystemBase{
  SparkFlex motor;
  SoftLimitConfig softLimit;
  
  
  public Deepclimb(){
    motor = new SparkFlex(Constants.Deepclimb.deviceID, MotorType.kBrushless);
    softLimit = new SoftLimitConfig();
  }
  public void spin(double speed){
    motor.set(speed);

  }
  public void liftArm(double angle){
    motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void pivot(double speed){
    
  }
  public void setSoftLimits(){
    // softLimit.forwardSoftLimitEnabled(true);
    // softLimit.reverseSoftLimitEnabled(true);
    // softLimit.forwardSoftLimit(Constants.Deepclimb.forwardSoftLimit);
    // softLimit.reverseSoftLimit(Constants.Deepclimb.reverseSoftLimit);
    
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