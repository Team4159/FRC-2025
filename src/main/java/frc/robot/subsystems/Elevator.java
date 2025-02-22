package frc.robot.subsystems;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.ElevatorState;

public class Elevator extends SubsystemBase{
    private SparkFlex motor;
    //the position the elevator is currently trying to get to
    private double targetPosition;
    //a planned position that can be stored for future use
    private double futureTarget;

    private DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    private ElevatorSim elevatorSim = new ElevatorSim(
        elevatorGearbox, 
        Constants.Elevator.elevatorGearing, 
        Constants.Elevator.elevatorWeightKG, 
        Constants.Elevator.spoolDiameter, 
        0, 
        Constants.Elevator.maxHeight, 
        true, 
        0, 
        0.001,
        0);

    //private SparkFlexSim motorSim;

    public Elevator(){
        motor = new SparkFlex(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
        //motorSim = new SparkFlexSim(motor, elevatorGearbox);
        //setGoalState(ElevatorState.STOW);
        setFutureState(ElevatorState.INTAKE);
        goToFutureState();
    }

    /** @param position the desired final state of the elevator */
    public void setGoalState(ElevatorState desiredState){
        Constants.Elevator.elevatorPID.setGoal(desiredState.height);
    }

    /** @param position the desired final state of the elevator */
    public void setFutureState(ElevatorState desiredState){
        SmartDashboard.putString("Elevator Mode", desiredState.name());
        futureTarget = desiredState.height;
    }
    
    public void goToFutureState(){
        //targetPosition = futureTarget;
        Constants.Elevator.elevatorPID.setGoal(futureTarget);
    }

    @Override
    public void periodic(){
        //System.out.println(elevatorSim.getPositionMeters());
        //System.out.println(targetPosition);
        //Constants.Elevator.elevatorPID.setGoal(targetPosition);
        double PIDOutput = Constants.Elevator.elevatorPID.calculate(motor.getEncoder().getPosition()/Constants.Elevator.rotationsPerMeter);
        double FFOutput = Constants.Elevator.elevatorFF.calculate(Constants.Elevator.elevatorPID.getSetpoint().velocity);
        motor.setVoltage(PIDOutput + FFOutput);
        SmartDashboard.putNumber("encoderPosition", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("height", getHeight());
        SmartDashboard.putNumber("percent", motor.getAppliedOutput());
        SmartDashboard.putNumber("desiredPosition", targetPosition);
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

    /** will return 0 if not used during simulation 
     * @return meters
    */
    public double getSimPosition(){
        if(RobotBase.isReal()) return 0;
        return elevatorSim.getPositionMeters();
    }

    public class ChangeState extends Command{
        private Constants.Elevator.ElevatorState elevatorState;
        private boolean continuous;

        // public ChangeState(Constants.Elevator.ElevatorState elevatorState){
        //     this.elevatorState = elevatorState;
        //     addRequirements(Elevator.this);
        // }

        /** @param elevatorState desired state for the elevator 
         * @param continuous if true the command will not end, if false the command will end when at desired position
        */
        public ChangeState(Constants.Elevator.ElevatorState elevatorState, boolean continuous){
            this.elevatorState = elevatorState;
            this.continuous = continuous;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize(){
            Elevator.this.setGoalState(elevatorState);
        }

        @Override
        public boolean isFinished(){
            return !continuous && MathUtil.isNear(getHeight(), targetPosition, Constants.Elevator.elevatorTolerance);
        }
    }
}
