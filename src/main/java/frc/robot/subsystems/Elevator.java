package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.ElevatorState;

public class Elevator extends SubsystemBase{
    private SparkFlex motor;

    private boolean zeroMode;

    //sim
    // private DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    // private ElevatorSim elevatorSim = new ElevatorSim(
    //     elevatorGearbox, 
    //     Constants.Elevator.elevatorGearing, 
    //     Constants.Elevator.elevatorWeightKG, 
    //     Constants.Elevator.spoolDiameter, 
    //     0, 
    //     Constants.Elevator.maxHeight, 
    //     true, 
    //     0, 
    //     0.001,
    //     0);

    //private SparkFlexSim motorSim;

    public Elevator(){
        motor = new SparkFlex(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
        setGoalState(ElevatorState.INTAKE);
    }

    /** @param position the desired final state of the elevator */
    public void setGoalState(ElevatorState desiredState){
        Constants.Elevator.elevatorPID.reset(getHeight());
        Constants.Elevator.elevatorPID.setGoal(desiredState.height);
        SmartDashboard.putNumber("desiredPosition", desiredState.height);
    }

    @Override
    public void periodic(){
        if(!zeroMode && DriverStation.isEnabled()){
            double PIDOutput = Constants.Elevator.elevatorPID.calculate(getHeight());
            double FFOutput = Constants.Elevator.elevatorFF.calculate(Constants.Elevator.elevatorPID.getSetpoint().velocity);
            motor.setVoltage((PIDOutput + FFOutput));
            if(motor.getReverseLimitSwitch().isPressed()){
                motor.getEncoder().setPosition(0);
            }
        }
        else{
            motor.set(-0.2);
            if(motor.getReverseLimitSwitch().isPressed()){
                zeroMode = false;
                motor.getEncoder().setPosition(0);
                setGoalState(ElevatorState.INTAKE);
            }
        }
        SmartDashboard.putNumber("encoderPosition", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("height", getHeight());
        SmartDashboard.putNumber("percent", motor.getAppliedOutput());
        SmartDashboard.putBoolean("limit switch", motor.getReverseLimitSwitch().isPressed());
    }

    public void toggleZeroElevator(){
        zeroMode = !zeroMode;
    }

    /** @return height of elevator in meters */
    public double getHeight(){
        return motor.getEncoder().getPosition() / Constants.Elevator.rotationsPerMeter;
    }

    // public void simulationPeriodic() {
    //     elevatorSim.setInput(motorSim.getSetpoint() * RobotController.getBatteryVoltage());
    //     //System.out.println(motorSim.getSetpoint());
    //     //System.out.println(.getPositionMeters());
    //      // In this method, we update our simulation of what our elevator is doing
    //     // First, we set our "inputs" (voltages)
    //     motorSim.iterate( // motor velocity, in RPM
    //         elevatorSim.getVelocityMetersPerSecond() * Constants.Elevator.rotationsPerMeter * 60,
    //         RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
    //         0.02); // Time interval, in Seconds

    //     // Next, we update it. The standard loop time is 20ms.
    //     elevatorSim.update(0.020);

    //     // Finally, we set our simulated encoder's readings and simulated battery voltage
    //     motorSim.setPosition(elevatorSim.getPositionMeters() * Constants.Elevator.rotationsPerMeter);
    //     // SimBattery estimates loaded battery voltages
    //     RoboRioSim.setVInVoltage(
    //         BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    // }

    // /** will return 0 if not used during simulation 
    //  * @return meters
    // */
    // public double getSimPosition(){
    //     if(RobotBase.isReal()) return 0;
    //     return elevatorSim.getPositionMeters();
    // }

    public class ChangeState extends Command{
        private Constants.Elevator.ElevatorState elevatorState;
        private boolean continuous;

        /** @param elevatorState desired state for the elevator 
        */
        public ChangeState(Constants.Elevator.ElevatorState elevatorState){
            this.elevatorState = elevatorState;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize(){
            Elevator.this.setGoalState(elevatorState);
        }

        @Override
        public boolean isFinished(){
            return !continuous && MathUtil.isNear(getHeight(), Constants.Elevator.elevatorPID.getGoal().position, Constants.Elevator.elevatorTolerance);
        }
    }
}
