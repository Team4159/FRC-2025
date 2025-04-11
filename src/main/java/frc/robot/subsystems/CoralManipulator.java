package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;

public class CoralManipulator extends SubsystemBase {
    // pivot 
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

    // private SparkFlexSim motorSim;
    // private SparkAbsoluteEncoderSim encoderSim;

    // roller
    public SparkMax rollerMotor;
    public DigitalInput beamBreak;

    public CoralManipulator() {

        //pivot
        angleMotor = new SparkFlex(Constants.CoralManipulator.angleMotorID, MotorType.kBrushless);
        // motorSim = new SparkFlexSim(angleMotor, gearbox);
        // encoderSim = motorSim.getAbsoluteEncoderSim();
        targetPosition = Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE.angle;

        //roller
        rollerMotor = new SparkMax(Constants.CoralManipulator.rollerMotorID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.CoralManipulator.beamBreakDIO);
        setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
    }

    @Override
    public void periodic(){
        //pivot
        if(DriverStation.isEnabled()){
            Constants.CoralManipulator.anglePID.setGoal(targetPosition);
            double pid = Constants.CoralManipulator.anglePID.calculate(getAngle());
            double ff;
            if(hasCoral()){
                //if there is a coral it will use the FF for coral
                ff = Constants.CoralManipulator.angleFFCoral.calculate(getAngle(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
            }
            else{
                //otherwise it will use the FF for when the manipulator is empty
                ff = Constants.CoralManipulator.angleFFEmpty.calculate(getAngle(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
            }
            angleMotor.setVoltage(ff + pid);
        }
        SmartDashboard.putNumber("armsetpoint", Units.radiansToDegrees(targetPosition));
        SmartDashboard.putNumber("armposition", Units.radiansToDegrees(getAngle()));

        //roller
        SmartDashboard.putBoolean("Coral", hasCoral());
    }

    public double getAngle(){
        return Units.rotationsToRadians(angleMotor.getAbsoluteEncoder().getPosition()) + Constants.CoralManipulator.FFOffset;
    }

    public boolean hasCoral(){
        return !beamBreak.get();
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
    public void setPivotGoalState(CoralManipulatorPivotState desiredState){
        targetPosition = desiredState.angle;
        Constants.CoralManipulator.anglePID.reset(getAngle());
    }

    /** @param desiredState the desired final state of coral manipulator roller */
    public void setRollerGoalState(CoralManipulatorRollerState desiredState){
        rollerMotor.set(desiredState.spin);
    }

    public class ChangePivotState extends Command {
        private CoralManipulatorPivotState state;

        public ChangePivotState(CoralManipulatorPivotState desiredState){
            state = desiredState;
            addRequirements(CoralManipulator.this);
        }

        @Override
        public void initialize(){
            CoralManipulator.this.setPivotGoalState(state);
        }

        @Override
        public boolean isFinished(){
            return MathUtil.isNear(getAngle(), targetPosition, Constants.CoralManipulator.angleTolerance);
        }
    }

    public class ChangeRollerState extends Command {
        private CoralManipulatorRollerState state;

        public ChangeRollerState(CoralManipulatorRollerState state){
            this.state = state;
            addRequirements(CoralManipulator.this);
        }

        @Override
        public void initialize(){
            CoralManipulator.this.setRollerGoalState(state);
        }

        @Override
        public void end(boolean interrupted){
            CoralManipulator.this.setRollerGoalState(CoralManipulatorRollerState.PASSIVE);
        }
    }

}