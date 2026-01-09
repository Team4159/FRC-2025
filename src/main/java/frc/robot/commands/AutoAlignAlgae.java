package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.ChangeState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PhotonVision;

public class AutoAlignAlgae extends Command{
    private AlgaeIntake algaeIntake;
    private CommandSwerveDrivetrain swerve;
    private LED led;
    private PhotonVision photonVision;
    private boolean finished;
    private double algaeAngle;

    public AutoAlignAlgae(CommandSwerveDrivetrain swerve, AlgaeIntake algaeIntake, LED led, PhotonVision photonVision){
        this.swerve = swerve;
        this.algaeIntake = algaeIntake;
        this.led = led;
        this.photonVision = photonVision;
        this.algaeAngle = 0;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        if(photonVision.getAlgaeYaw() == null) finished = true;
        else{
            led.blink(Color.kYellow, 0.25);
            algaeIntake.new ChangeState(AlgaeIntakeState.INTAKE).schedule();
        }
        algaeAngle = 0;
    }

    @Override
    public void execute(){
        //get algae yaw from PV
        Double currentYaw = photonVision.getAlgaeYaw();
        if(currentYaw != null) algaeAngle = Units.degreesToRadians(currentYaw.doubleValue());
        //robot relative translation (multiplied by -1 because algae is on the back of the robot)
        double xSpeed = -1 * Math.cos(algaeAngle) * Constants.Swerve.maxAutoIntakeAlgaeSpeed;
        double ySpeed = -1 * Math.sin(algaeAngle) * Constants.Swerve.maxAutoIntakeAlgaeSpeed;
        //robot relative rotation (try to get PV yaw to 0, may or may not need to multiply by -1 need to test)
        double omega = Constants.Swerve.rotationController.calculate(algaeAngle, 0);
        swerve.RobotRelativeDrive(xSpeed, ySpeed, omega);
        SmartDashboard.putNumber("algae angle", algaeAngle);
    }

    @Override
    public boolean isFinished(){
        if(finished)
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            led.blink(Color.kRed, 0.25);
        }
        swerve.stopSwerve();
        algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.STOW, true).schedule();
    }
}
