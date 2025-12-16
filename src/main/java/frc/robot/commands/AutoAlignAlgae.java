package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PhotonVision;

public class AutoAlignAlgae extends Command{
    private AlgaeIntake algaeIntake;
    private CommandSwerveDrivetrain swerve;
    private LED led;
    private PhotonVision photonVision;
    private boolean finished;

    public AutoAlignAlgae(CommandSwerveDrivetrain swerve, AlgaeIntake algaeIntake, LED led, PhotonVision photonVision){
        this.swerve = swerve;
        this.algaeIntake = algaeIntake;
        this.led = led;
        this.photonVision = photonVision;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        if(photonVision.getAlgaeYaw() == null) finished = true;
        else{
            led.blink(Color.kYellow, 0.25);
            algaeIntake.new ChangeState(AlgaeIntakeState.INTAKE).schedule();
        }
    }

    @Override
    public void execute(){
        double currentYaw = photonVision.getAlgaeYaw();
        double xSpeed = Math.sin(currentYaw) * Constants.Swerve.maxAutoIntakeAlgaeSpeed;
        double ySpeed = Math.cos(currentYaw) * Constants.Swerve.maxAutoIntakeAlgaeSpeed;
        double omega = Constants.Swerve.rotationController.calculate(currentYaw, 0);
        swerve.RobotRelativeDrive(xSpeed, ySpeed, omega);
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
    }
}
