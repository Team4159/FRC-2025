package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AdjustableSlewRateLimiter;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.Drive;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double maxAccel;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    //private final SwerveRequest.ApplyFieldSpeeds m_driveApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(3, 0, 0);
    private final PIDController m_pathYController = new PIDController(3, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(5, 0, 0);

    private final PIDController m_vxController = new PIDController(1, 0, 0);
    private final PIDController m_vyController = new PIDController(1, 0, 0);
    private final PIDController m_omegaController = new PIDController(5, 0, 0){{
        enableContinuousInput(-Math.PI, Math.PI);
    }};

    private QuestNav questNav = new QuestNav();

    //private double desiredYaw;

    private final SwerveRequest.ApplyFieldSpeeds m_driveApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_driveApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final Field2d closestPoseF2d = new Field2d();
    private final Field2d oculusF2d = new Field2d();
    private Pose2d closestAutoAlignPose = new Pose2d();
    private Pose2d secondClosestAutoAlignPose = new Pose2d();
    private Pose2d closestStationAutoAlignPose = new Pose2d();

    private AdjustableSlewRateLimiter limiterX = new AdjustableSlewRateLimiter(1, -1, 0);
    private AdjustableSlewRateLimiter limiterY = new AdjustableSlewRateLimiter(1, -1, 0);

    private Elevator elevator;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1.5).per(Second),        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            Seconds.of(5),        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

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
        questNav.zeroHeading();
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
            //() -> getOculusPose(),
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
        questNav.resetPosition(newPose);
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;
        //var pose = getOculusPose();

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
        // if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)){
        //     speedX *= -1;
        //     speedY *= -1;
        // }
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speedX, speedY, 0);
        setControl(m_driveApplyRobotSpeeds.withSpeeds(desiredSpeeds));
    }

    /** uses field relative control */
    public void drive(double inputX, double inputY, double inputOmega){
        //System.out.println(maxAccel);
        //desiredYaw += inputOmega*0.02;
        //desiredYaw = MathUtil.angleModulus(desiredYaw);
        double speedX = limiterX.calculate(MathUtil.applyDeadband(inputX, 0.1)* TunerConstants.kSpeedAt12Volts.magnitude());
        double speedY = limiterY.calculate(MathUtil.applyDeadband(inputY, 0.1) * TunerConstants.kSpeedAt12Volts.magnitude());
        //double speedT = m_omegaController.calculate(getState().Pose.getRotation().getRadians(), desiredYaw);
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)){
            speedX *= -1;
            speedY *= -1;
        }
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speedX, speedY, inputOmega* 4);
        setControl(m_driveApplyFieldSpeeds.withSpeeds(desiredSpeeds));
        
    }

    public void stopSwerve(){
        //setControl(m_pathApplyFieldSpeeds.withSpeeds(new ChassisSpeeds()));
        System.out.println("stopping");
    }

    public void calculateClosestReef(){
        Translation2d reefTranslation;
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 - Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        else
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 + Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        Pose2d processor = Constants.Field.processors.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        if(getState().Pose.getTranslation().getDistance(reefTranslation) < Constants.Swerve.maxReefAutoAlignDistatnce){
        //if(getOculusPose().getTranslation().getDistance(reefTranslation) < Constants.Swerve.maxReefAutoAlignDistatnce){
            var reefPoses = Constants.Field.reef.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
            var closestSide = getState().Pose.nearest(reefPoses);
            //var closestSide = getOculusPose().nearest(reefPoses);
            var angle = closestSide.getRotation().getRadians() - Math.PI/2;
            //closestAutoAlignPose = new Pose2d(closestSide.getX() + Units.inchesToMeters(Constants.Field.middletoPole) * Math.cos(angle), closestSide.getY() + Units.inchesToMeters(Constants.Field.middletoPole) * Math.sin(angle), closestSide.getRotation());
            //secondClosestAutoAlignPose = new Pose2d(closestSide.getX() - Units.inchesToMeters(Constants.Field.middletoPole) * Math.cos(angle), closestSide.getY() - Units.inchesToMeters(Constants.Field.middletoPole) * Math.sin(angle), closestSide.getRotation());
            var p1 = new Pose2d(closestSide.getX() + Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() + Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            var p2 = new Pose2d(closestSide.getX() - Constants.Field.middletoPole * Math.cos(angle), closestSide.getY() - Constants.Field.middletoPole * Math.sin(angle), closestSide.getRotation());
            if(p1.getTranslation().getDistance(getState().Pose.getTranslation()) < p2.getTranslation().getDistance(getState().Pose.getTranslation())){
            //if(p1.getTranslation().getDistance(getOculusPose().getTranslation()) < p2.getTranslation().getDistance(getOculusPose().getTranslation())){
                closestAutoAlignPose = p1;
                secondClosestAutoAlignPose = p2;
            }
            else{
                closestAutoAlignPose = p2;
                secondClosestAutoAlignPose = p1;
            }
        }
        else if(getState().Pose.getTranslation().getDistance(processor.getTranslation()) < Constants.Swerve.maxReefAutoAlignDistatnce){
        //else if(getOculusPose().getTranslation().getDistance(processor.getTranslation()) < Constants.Swerve.maxReefAutoAlignDistatnce){
            closestAutoAlignPose = processor;
        }
        else{
            var reefPoses = Constants.Field.stations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
            closestAutoAlignPose = getState().Pose.nearest(reefPoses);
            //closestAutoAlignPose = getOculusPose().nearest(reefPoses);
        }
    }

    public Pose2d getDesieredAutoAlignPose(boolean L4, boolean useSecondClosestPose){
        // if(L4){
        //     var angle = closestAutoAlignPose.getRotation().getRadians() - Math.PI;
        //     closestAutoAlignPose = new Pose2d(closestAutoAlignPose.getX() + Constants.Swerve.L4Offset * Math.cos(angle), closestAutoAlignPose.getY() + Constants.Swerve.L4Offset * Math.sin(angle), closestAutoAlignPose.getRotation());
        //     secondClosestAutoAlignPose = new Pose2d(secondClosestAutoAlignPose.getX() + Constants.Swerve.L4Offset * Math.cos(angle), secondClosestAutoAlignPose.getY() + Constants.Swerve.L4Offset * Math.sin(angle), secondClosestAutoAlignPose.getRotation());
        // }
        if(useSecondClosestPose){
            return secondClosestAutoAlignPose;
        }
        return closestAutoAlignPose;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

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
        //closestPoseF2d.setRobotPose(getOculusPose());
        oculusF2d.setRobotPose(questNav.getPose());
        SmartDashboard.putData("Oculus Pose", oculusF2d);
        SmartDashboard.putData("closest reef", closestPoseF2d);
        SmartDashboard.putBoolean("Oculus connected", questNav.connected());
        setMaxAccel();
        setVisionMeasurementStdDevs(VecBuilder.fill(0, 0, 0));
        addVisionMeasurement(questNav.getPose(), questNav.timestamp());
    }

    public Pose2d getOculusPose(){
        return questNav.getPose();
    }

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

    public class Drive extends Command{
        CommandPS4Controller ps4Controller;
        CommandJoystick joystick;
        public Drive(CommandPS4Controller ps4Controller){
            this.ps4Controller = ps4Controller;
            addRequirements(CommandSwerveDrivetrain.this);
        }
        public Drive(CommandJoystick joystick){
            this.joystick = joystick;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            if(ps4Controller != null){
                drive(MathUtil.applyDeadband(-ps4Controller.getLeftY(), 0.1), MathUtil.applyDeadband(-ps4Controller.getLeftX(), 0.1), MathUtil.applyDeadband(-ps4Controller.getRightX(), 0.1));
            }
            else{
                drive(-joystick.getY(), -joystick.getX(), -joystick.getZ());
            }
        }
    }

    public class ManualAlign extends Command{
        double inputX, inputY;
        CommandJoystick joystick;
        public ManualAlign(CommandJoystick joystick, double inputX, double inputY){
            this.joystick = joystick;
            this.inputX = inputX;
            this.inputY = inputY;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        public void execute(){
            ManualAlign(inputX, inputY);
        }
    }
}
