// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Ultrasonic;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class ManualDSAlign extends Command{
//     private final SwerveRequest.ApplyRobotSpeeds m_driveApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
//     private final PIDController xController = new PIDController(1, 0, 0);
//     private CommandSwerveDrivetrain drivetrain;
//     private Ultrasonic distanceSensor;
//     private double speedY;

//     public ManualDSAlign(CommandSwerveDrivetrain drivetrain, double speedY){
//         this.drivetrain = drivetrain;
//         this.speedY = speedY;
//         distanceSensor = new Ultrasonic(Constants.Swerve.distanceSensorPingDIO, Constants.Swerve.distanceSensorEchoDIO);
//     }

//     @Override
//     public void initialize(){
//         xController.reset();
//     }

//     @Override
//     public void execute(){
//         double speedX = -xController.calculate(distanceSensor.getRangeInches(), Constants.Swerve.distSensorAutoAlignDistInches);
//         ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, 0);
//         drivetrain.setControl(m_driveApplyRobotSpeeds.withSpeeds(speeds));
//     }
// }
