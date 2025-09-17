package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AdjustableSlewRateLimiter;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    /** Maximum acceleration allowed to prevent tipping.
     * The maximum acceleration decreases linearly as the elevator extends due to higher center of gravity
     */
    private double maxAccel;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following 
     * closed loop control
    */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    //private final SwerveRequest.ApplyFieldSpeeds m_driveApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(4, 0, 0);
    private final PIDController m_pathYController = new PIDController(4, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(4, 0, 0);

    //private double desiredYaw;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity 0.75 old value

    /** Field Centric closed loop request for teleop driving */
    private final SwerveRequest.FieldCentric m_fieldCentricClosedLoopRequest = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // use closed loop control for drive motors
    /** Robot relative request for manual robot relative align */
    private final SwerveRequest.ApplyRobotSpeeds m_driveApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //is also a map of the robots position on field
    private final Field2d closestPoseF2d = new Field2d();
    private Pose2d closestAutoAlignPose = new Pose2d();

    private AdjustableSlewRateLimiter limiterX = new AdjustableSlewRateLimiter(1, -1, 0);
    private AdjustableSlewRateLimiter limiterY = new AdjustableSlewRateLimiter(1, -1, 0);

    private Elevator elevator;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         Volts.of(1.5).per(Second),        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         Seconds.of(5),        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    /* The SysId routine to test */
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        zero();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        zero();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        zero();
    }

    public void zero(){
        Rotation2d offset = new Rotation2d();
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) offset = new Rotation2d(Math.PI);
        resetPose(new Pose2d(getState().Pose.getTranslation(), offset));
        //desiredYaw = offset.getRadians();
    }
    
    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void setElevator(Elevator elevator){
        this.elevator = elevator;
    }

    @Override 
    public void resetPose(Pose2d newPose){
        super.resetPose(newPose);
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        //log auto error to determine source of unreliability
        SmartDashboard.putNumber("Auto X error", pose.getX() - sample.x);
        SmartDashboard.putNumber("Auto Y error", pose.getY() - sample.y);
        SmartDashboard.putNumber("Auto Theta error", pose.getRotation().getRadians() - sample.heading);

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /** uses robot relative control */
    public void ManualAlign(double inputX, double inputY){
        double speedX = limiterX.calculate(MathUtil.applyDeadband(inputX, 0.1)* TunerConstants.kSpeedAt12Volts.magnitude());
        double speedY = limiterY.calculate(MathUtil.applyDeadband(inputY, 0.1) * TunerConstants.kSpeedAt12Volts.magnitude());
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speedX, speedY, 0);
        setControl(m_driveApplyRobotSpeeds.withSpeeds(desiredSpeeds));
    }

    /** uses field relative control */
    public void drive(double inputX, double inputY, double inputOmega){
        //open loop
        // double speedX = limiterX.calculate(MathUtil.applyDeadband(inputX * MaxSpeed, 0.1));
        // double speedY = limiterY.calculate(MathUtil.applyDeadband(inputY * MaxSpeed, 0.1));
        // ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speedX, speedY, inputOmega* 4);
        // setControl(m_driveApplyFieldSpeeds.withSpeeds(desiredSpeeds));

        //closed loop
        double desiredSpeedX = limiterX.calculate(inputX * MaxSpeed);
        double desiredSpeedY = limiterY.calculate(inputY * MaxSpeed);
        setControl(m_fieldCentricClosedLoopRequest.withVelocityX(desiredSpeedX).withVelocityY(desiredSpeedY).withRotationalRate(inputOmega * MaxAngularRate));
        
    }

    public void stopSwerve(){
        setControl(m_pathApplyFieldSpeeds.withSpeeds(new ChassisSpeeds()));
    }

    /** calculates the closest reef pole to the robot */
    public void calculateClosestReef(){
        Translation2d reefTranslation;
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 - Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        else
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 + Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        if(getState().Pose.getTranslation().getDistance(reefTranslation) < Constants.Swerve.maxReefAutoAlignDistatnce){
            var reefPoses = Constants.Field.reef.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
            var closestSide = getState().Pose.nearest(reefPoses);
            var angle = closestSide.getRotation().getRadians() - Math.PI/2;
            var p1 = new Pose2d(closestSide.getX() + Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() + Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            var p2 = new Pose2d(closestSide.getX() - Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() - Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            if(p1.getTranslation().getDistance(getState().Pose.getTranslation()) < p2.getTranslation().getDistance(getState().Pose.getTranslation())){
                closestAutoAlignPose = p1;
            }
            else{
                closestAutoAlignPose = p2;
            }
        }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.quasistatic(direction);
    // }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        calculateClosestReef();
        closestPoseF2d.getObject("closestPose").setPose(closestAutoAlignPose);
        closestPoseF2d.setRobotPose(getState().Pose);
        SmartDashboard.putData("closest reef", closestPoseF2d);
        setMaxAccel();
    }

    /**
     * Sets maximum acceleration based on height of the elevator using linear interpolation.
     */
    private void setMaxAccel(){
        if(elevator != null){
            maxAccel = MathUtil.interpolate(Constants.Swerve.maxAccelFullRetraction, Constants.Swerve.maxAccelFullExtension, elevator.getHeight() / Constants.Elevator.maxHeight);
            limiterX.setRateLimits(maxAccel, -maxAccel);
            limiterY.setRateLimits(maxAccel, -maxAccel);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public class DriveJoystick extends Command{
        CommandJoystick joystick;
        /**
         * This command is for field-relative driving in teleop.
         * It uses closed loop for control.
         * @param joystick CommandJoystick to be used for driving the robot.
         */
        public DriveJoystick(CommandJoystick joystick){
            this.joystick = joystick;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            drive(-joystick.getY(), -joystick.getX(), -joystick.getZ());
        }
    }

    public class DriveXbox extends Command{
        CommandXboxController commandXboxController;
        /**
         * This command is for field-relative driving in teleop.
         * It uses closed loop for control.
         * It is currently only used for at-home simulation.
         * @param commandPS4Controller CommandPS4Controller used for driving the robot
         */
        public DriveXbox(CommandXboxController commandXboxController){
            this.commandXboxController = commandXboxController;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            drive(-Math.signum(commandXboxController.getLeftY()) * Math.pow(commandXboxController.getLeftY(), 2),
             -Math.signum(commandXboxController.getLeftX()) * Math.pow(commandXboxController.getLeftX(), 2), 
             -commandXboxController.getRightX());
        }
    }

    public class DrivePS4 extends Command{
        CommandPS4Controller commandPS4Controller;
        /**
         * This command is for field-relative driving in teleop.
         * It uses closed loop for control.
         * It is currently only used for at-home simulation.
         * @param commandPS4Controller CommandPS4Controller used for driving the robot
         */
        public DrivePS4(CommandPS4Controller commandPS4Controller){
            this.commandPS4Controller = commandPS4Controller;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            drive(-commandPS4Controller.getRightY(), -commandPS4Controller.getRightX(), -commandPS4Controller.getLeftX());
        }
    }

    public class ManualAlign extends Command{
        double inputX, inputY;
        /**
         * This command is for robot-relative driving in teleop.
         * It is designed for alignment purposes and not regular driving.
         * @param inputX desired speed in X direction
         * @param inputX desired speed in Y direction
         */
        public ManualAlign(double inputX, double inputY){
            this.inputX = inputX;
            this.inputY = inputY;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            ManualAlign(inputX, inputY);
        }
    }
}
