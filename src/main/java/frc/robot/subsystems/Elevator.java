package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private SparkFlex motor;
    private double targetPosition;

    private DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    private ElevatorSim elevatorSim = new ElevatorSim(
        elevatorGearbox, 
        25, 
        10, 
        0.25, 
        0, 
        20, 
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
        SmartDashboard.putData("Elevator Sim", m_mech2d);
        motorSim = new SparkFlexSim(motor, elevatorGearbox);
        encoderSim = motorSim.getAbsoluteEncoderSim();
        setGoalPosition(0);
    }

    /** @param position meters */
    /**    takes a position in meters and sets the target position to an amount of rotations */
    public void setGoalPosition(double position){
        System.out.println("setting position");
        targetPosition = position * Constants.Elevator.rotationsPerMeter;
    }

    @Override
    public void periodic(){
        //System.out.println(elevatorSim.getPositionMeters());
        //System.out.println(targetPosition);
        Constants.Elevator.elevatorPID.setGoal(targetPosition);
        double PIDOutput = Constants.Elevator.elevatorPID.calculate(motor.getAbsoluteEncoder().getPosition());
        double FFOutput = Constants.Elevator.elevatorFF.calculate(Constants.Elevator.elevatorPID.getSetpoint().velocity);
        motor.set(PIDOutput + FFOutput);
        System.out.println(PIDOutput + FFOutput);
    }

    /** @return height of elevator in meters */
    public double getHeight(){
        return motor.getAbsoluteEncoder().getPosition() / Constants.Elevator.rotationsPerMeter;
    }

    public void simulationPeriodic() {
        elevatorSim.setInput(motorSim.getSetpoint() * RobotController.getBatteryVoltage());
        //System.out.println(motorSim.getSetpoint());
        //System.out.println(elevatorSim.getPositionMeters());
         // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            elevatorSim.getVelocityMetersPerSecond()),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds

        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setPosition(elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        updateTelemetry();
    }

    public void updateTelemetry() {
        // Update elevator visualization with position
        m_elevatorMech2d.setLength(10 + elevatorSim.getPositionMeters() );
    }
    public class ChangeState extends Command{
        private Constants.Elevator.ElevatorState elevatorState;
        private boolean continuous, stowOnEnd;

        /** @param elevatorState desired state for the elevator
        @param continuous if true the command will not end until interrupted
        @param stowOnEnd if true the elevator will return to stow position when the command ends*/
        public ChangeState(Constants.Elevator.ElevatorState elevatorState, boolean continuous, boolean stowOnEnd){
            this.elevatorState = elevatorState;
            addRequirements(Elevator.this);
        }

        /** @param elevatorState desired state for the elevator */
        public ChangeState(Constants.Elevator.ElevatorState elevatorState){
            this(elevatorState, false, false);
        }
        
        @Override
        public void initialize(){
            Elevator.this.setGoalPosition(elevatorState.height);
        }

        @Override
        public boolean isFinished(){
            //System.out.println(elevatorState.height);
            return false;
            //return !continuous && MathUtil.isNear(targetPosition, targetPosition, targetPosition);
        }

        @Override
        public void end(boolean interrupted){
            if(stowOnEnd) Elevator.this.setGoalPosition(Constants.Elevator.ElevatorState.STOW.height);
        }
    }
}
