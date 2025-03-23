package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;

public class CoralManipulatorPivot extends SubsystemBase{
    private SparkFlex angleMotor;
    private double targetPosition;
    private BooleanSupplier hasCoralSupplier;

    private LQR lqr;

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

    public CoralManipulatorPivot(BooleanSupplier hasCoralSupplier){
        angleMotor = new SparkFlex(Constants.CoralManipulator.angleMotorID, MotorType.kBrushless);
        // motorSim = new SparkFlexSim(angleMotor, gearbox);
        // encoderSim = motorSim.getAbsoluteEncoderSim();
        targetPosition = Constants.CoralManipulator.CoralManipulatorPivotState.INTAKE.angle;
        this.hasCoralSupplier = hasCoralSupplier;
        lqr = this.new LQR();
        lqr.teleopInit();
    }

    @Override
    public void periodic(){
        if (hasCoralSupplier.getAsBoolean()) {
            lqr.teleopPeriodic();
            return;
        }

        if(DriverStation.isEnabled()){
            Constants.CoralManipulator.anglePID.setGoal(targetPosition);
            double pid = Constants.CoralManipulator.anglePID.calculate(getAngle());
            double ff;
            if(hasCoralSupplier.getAsBoolean()){
                //if there is a coral it will use the FF for coral
                ff = Constants.CoralManipulator.angleFFCoral.calculate(getAngle(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
            }
            else{
                //otherwise it will use the FF for when the manipulator is empty
                ff = Constants.CoralManipulator.angleFFEmpty.calculate(getAngle(), Constants.CoralManipulator.anglePID.getSetpoint().velocity);
            }
            angleMotor.setVoltage(ff + pid);
            System.out.println(Constants.CoralManipulator.anglePID.getSetpoint().velocity);
        }
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
            //System.out.println("coralmanip: " + MathUtil.isNear(getAngle(), targetPosition, Constants.CoralManipulator.angleTolerance));
            return !continuous && MathUtil.isNear(getAngle(), targetPosition, Constants.CoralManipulator.angleTolerance);
        }
    }

    private class LQR {
          // Moment of inertia of the arm, in kg * m^2. Can be estimated with CAD. If finding this constant
  // is difficult, LinearSystem.identifyPositionSystem may be better.
  private static final double kArmMOI = 0.356734613;

  // Reduction between motors and encoder, as output over input. If the arm spins slower than
  // the motors, this number should be greater than one.
  private static final double kArmGearing = 60.0;

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              6, // radians / second
              10)); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // The plant holds a state-space model of our arm. This system has the following properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N2> m_armPlant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), kArmMOI, kArmGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  @SuppressWarnings("unchecked")
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
          VecBuilder.fill(0.015, 0.17), // How accurate we
          // think our model is, in radians and radians/sec
          VecBuilder.fill(0.01), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          0.020);

  // A LQR uses feedback to create voltage commands.
  @SuppressWarnings("unchecked")
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
          VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
          // Position and velocity error tolerances, in radians and radians per second. Decrease
          // this
          // to more heavily penalize state excursion, or make the controller behave more
          // aggressively. In this example we weight position much more highly than velocity, but
          // this
          // can be tuned to balance the two.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  @SuppressWarnings("unchecked")
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(
          (LinearSystem<N2, N1, N1>) m_armPlant.slice(0), m_controller, m_observer, 12.0, 0.020);



  private double getAngle() {
    return Units.rotationsToRadians(angleMotor.getAbsoluteEncoder().getPosition());
  }

  private double getRate() {
    return Units.rotationsPerMinuteToRadiansPerSecond(angleMotor.getEncoder().getVelocity());
  }

  public void teleopInit() {
    // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(getAngle(), getRate()));

    // Reset our last reference to the current state.
    m_lastProfiledReference =
        new TrapezoidProfile.State(getAngle(), getRate());
  }

  public void teleopPeriodic() {
    // Sets the target position of our arm. This is similar to setting the setpoint of a
    // PID controller.
    TrapezoidProfile.State goal = new TrapezoidProfile.State(targetPosition, 0.0);
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, goal);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(getAngle()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    angleMotor.setVoltage(nextVoltage);
  }
    }
}
