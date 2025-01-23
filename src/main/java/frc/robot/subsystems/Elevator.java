package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private SparkFlex motor;
    private DigitalInput limitSwitch;
    private double targetPosition;
    private double timeOffset;
    private TrapezoidProfile.State currentState, endState;

    private DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    private ElevatorSim elevatorSim = new ElevatorSim(
        elevatorGearbox, 
        25, 
        10, 
        1, 
        0, 
        1, 
        true, 
        0, 
        0.01,
        0);

    private SparkFlexSim motorSim;
    private SparkAbsoluteEncoderSim encoderSim;

    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

    public Elevator(){
        motor = new SparkFlex(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(Constants.Elevator.limitSwitchPort);
        SmartDashboard.putData("Elevator Sim", m_mech2d);
        motorSim = new SparkFlexSim(motor, elevatorGearbox);
        encoderSim = new SparkAbsoluteEncoderSim(motor);
        setGoalPosition(0);
    }

    public void setGoalPosition(double position){
        timeOffset = Timer.getFPGATimestamp();
        currentState = new TrapezoidProfile.State(motor.getEncoder().getPosition(), motor.getEncoder().getVelocity());
        endState = new TrapezoidProfile.State(position, 0);
    }

    @Override
    public void periodic(){
        var state = Constants.Elevator.trapezoidProfile.calculate(Timer.getFPGATimestamp()-timeOffset, currentState, endState);
        motor.set(
            Constants.Elevator.elevatorPID.calculate(motor.getEncoder().getPosition(), state.position)
            + Constants.Elevator.elevatorFF.calculate(state.velocity));

        if(limitSwitch.get()){
            motor.getEncoder().setPosition(0);
        }
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInput(motorSim.getVelocity() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setPosition(elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    public void updateTelemetry() {
        // Update elevator visualization with position
        m_elevatorMech2d.setLength(encoderSim.getPosition());
    }
}
