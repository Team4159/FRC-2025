package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;

public class CoralManipulatorPivot extends SubsystemBase{
    private SparkFlex angleMotor;
    private double targetPosition;

    //simulation
    private DCMotor gearbox = DCMotor.getNeoVortex(1);
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        gearbox, 
        Constants.CoralManipulator.gearRatio, 
        Constants.CoralManipulator.MOI, 
        Constants.CoralManipulator.lengthMeters, 
        Units.degreesToRadians(-80), 
        Units.degreesToRadians(270), 
        true, 
        Units.degreesToRadians(0), 
        0, 0);

    private SparkFlexSim motorSim;
    private SparkAbsoluteEncoderSim encoderSim;

    public CoralManipulatorPivot(){
        angleMotor = new SparkFlex(Constants.CoralManipulator.angleMotorID, MotorType.kBrushless);
        motorSim = new SparkFlexSim(angleMotor, gearbox);
        encoderSim = motorSim.getAbsoluteEncoderSim();
        targetPosition = Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE.angle;
    }

    @Override
    public void periodic(){
        Constants.CoralManipulator.anglePID.setGoal(targetPosition);
        double pid = Constants.CoralManipulator.anglePID.calculate(angleMotor.getAbsoluteEncoder().getPosition());
        double ff = Constants.CoralManipulator.angleFF.calculate(angleMotor.getAbsoluteEncoder().getPosition(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
        //System.out.println(Constants.CoralManipulator.anglePID.getSetpoint().velocity);
        angleMotor.set(ff + pid);
        //System.out.println(ff);
        System.out.println(Units.radiansToDegrees(angleMotor.getAbsoluteEncoder().getPosition()));
        //System.out.println(armSim.getAngleRads());
    }

    public void simulationPeriodic(){
        armSim.setInput(motorSim.getSetpoint() * RobotController.getBatteryVoltage());
        //System.out.println(motorSim.getSetpoint());
        //System.out.println(elevatorSim.getPositionMeters());
         // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            armSim.getVelocityRadPerSec()),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setPosition(armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    /** will return 0 if not used during simulation 
     * @return radians
    */
    public double getSimPosition(){
        if(RobotBase.isReal()) return 0;
        return armSim.getAngleRads() - Math.PI/2;
    }

    public void setGoalPosition(double position){
        targetPosition = position;
    }

    public class ChangeState extends Command{
        private CoralManipulatorPivotState state;

        public ChangeState(CoralManipulatorPivotState desiredState){
            state = desiredState;
            addRequirements(CoralManipulatorPivot.this);
        }

        @Override
        public void initialize(){
            CoralManipulatorPivot.this.setGoalPosition(state.angle);
        }
    }
}
