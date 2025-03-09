package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;

public class CoralManipulatorPivot extends SubsystemBase{
    private SparkFlex angleMotor;
    private double targetPosition;
    private boolean isL4;

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

    // private SparkFlexSim motorSim;
    // private SparkAbsoluteEncoderSim encoderSim;

    public CoralManipulatorPivot(){
        angleMotor = new SparkFlex(Constants.CoralManipulator.angleMotorID, MotorType.kBrushless);
        // motorSim = new SparkFlexSim(angleMotor, gearbox);
        // encoderSim = motorSim.getAbsoluteEncoderSim();
        targetPosition = Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE.angle;
        isL4 = false;
    }

    @Override
    public void periodic(){
        Constants.CoralManipulator.anglePID.setGoal(targetPosition);
        double pid = Constants.CoralManipulator.anglePID.calculate(getAngle());
        double ff = Constants.CoralManipulator.angleFF.calculate(getAngle(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
        angleMotor.setVoltage(ff + pid);
        SmartDashboard.putNumber("armsetpoint", Units.radiansToDegrees(targetPosition));
        SmartDashboard.putNumber("armposition", Units.radiansToDegrees(getAngle()));
    }

    public double getAngle(){
        return Units.rotationsToRadians(angleMotor.getAbsoluteEncoder().getPosition()) + Constants.CoralManipulator.FFOffset;
    }

    // public void simulationPeriodic(){
    //     armSim.setInput(motorSim.getSetpoint() * RobotController.getBatteryVoltage());
    //     //System.out.println(motorSim.getSetpoint());
    //     //System.out.println(elevatorSim.getPositionMeters());
    //      // In this method, we update our simulation of what our elevator is doing
    //     // First, we set our "inputs" (voltages)
    //     motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
    //         armSim.getVelocityRadPerSec()),
    //         RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
    //         0.02); // Time interval, in Seconds

    //     // Next, we update it. The standard loop time is 20ms.
    //     armSim.update(0.020);

    //     // Finally, we set our simulated encoder's readings and simulated battery voltage
    //     encoderSim.setPosition(Units.radiansToRotations(armSim.getAngleRads()));
    //     // SimBattery estimates loaded battery voltages
    //     RoboRioSim.setVInVoltage(
    //         BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    // }

    /** will return 0 if not used during simulation 
     * @return radians
    */
    public double getSimPosition(){
        if(RobotBase.isReal()) return 0;
        return armSim.getAngleRads() - Math.PI/2;
    }

    /** @param desiredState desired final state of coral manipulator pivot */
    public void setGoalState(CoralManipulatorPivotState desiredState){
        targetPosition = desiredState.angle;
        Constants.CoralManipulator.anglePID.reset(getAngle());
        if(desiredState.equals(CoralManipulatorPivotState.L4SETUP)){
            isL4 = true;
        }
        else{
            isL4 = false;
        }
    }

    public boolean isL4(){
        return isL4;
    }

    public class ChangeState extends Command{
        private CoralManipulatorPivotState state;
        private boolean continuous;

        public ChangeState(CoralManipulatorPivotState desiredState, boolean continuous){
            state = desiredState;
            this.continuous = continuous;
            addRequirements(CoralManipulatorPivot.this);
        }

        @Override
        public void initialize(){
            CoralManipulatorPivot.this.setGoalState(state);
        }

        @Override
        public boolean isFinished(){
            return !continuous && MathUtil.isNear(Units.rotationsToRadians(angleMotor.getAbsoluteEncoder().getPosition()), targetPosition, Constants.CoralManipulator.angleTolerance);
        }
    }
}
