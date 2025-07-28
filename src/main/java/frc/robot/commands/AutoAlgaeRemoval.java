package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;

public class AutoAlgaeRemoval extends AutoSwerve{
    private Elevator  elevator;
    private CoralManipulator coralManipulator;
    private LED led;
    private Pose2d midPose, finalPose;
    private boolean beginningState, tooFar;

    public AutoAlgaeRemoval(CommandSwerveDrivetrain drivetrain, Elevator elevator, CoralManipulator coralManipulator, LED led){
        super(drivetrain);
        this.elevator = elevator;
        this.coralManipulator = coralManipulator;
        this.led = led;

        //addRequirements(elevator, coralManipulator, led);
    }
    
    @Override
    public void initialize(){
        Translation2d reefTranslation;
        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue))
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 - Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        else
            reefTranslation = new Translation2d(Constants.Field.fieldLength/2 + Constants.Field.reefDistFromCenter, Constants.Field.fieldWidth/2);
        if(swerve.getState().Pose.getTranslation().getDistance(reefTranslation) < Constants.Swerve.maxReefAutoAlignDistatnce){
            led.blink(Color.kYellow, 0.25);
            var reefPoses = Constants.Field.reef.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
            var basePose = swerve.getState().Pose.nearest(reefPoses);
            var angle = basePose.getRotation().getRadians() - Math.PI/2;
            midPose =  new Pose2d(basePose.getX() + Constants.Field.autoAlgaeRemovalMidPoseDistance * Math.sin(angle), basePose.getY() - Constants.Field.autoAlgaeRemovalMidPoseDistance * Math.cos(angle), basePose.getRotation());
            finalPose = new Pose2d(basePose.getX() + Constants.Field.autoAlgaeRemovalFinalPoseDistance * Math.sin(angle), basePose.getY() - Constants.Field.autoAlgaeRemovalFinalPoseDistance * Math.cos(angle), basePose.getRotation());

            led.blink(Color.kYellow, 0.25);
            beginningState = true;
            tooFar = false;
            desiredPose = midPose;
            elevator.setGoalState(ElevatorState.L2);
            coralManipulator.setPivotGoalState(CoralManipulatorPivotState.L2);
            coralManipulator.setRollerGoalState(CoralManipulatorRollerState.OUTTAKE);
            super.initialize();
        }
        else{
            tooFar = true;
        }
    }

    @Override
    public void execute(){
        if(tooFar){
            return;
        }
        else if(beginningState){
            if(swerveAtGoal() && subsystemsAtGoal()){
                beginningState = false;
                desiredPose = finalPose;
            }
        }
        super.execute();
    }

    @Override
    public boolean isFinished(){
        return tooFar || swerveAtGoal() && !beginningState;
    }

    private boolean subsystemsAtGoal(){
        if(MathUtil.isNear(elevator.getHeight(), ElevatorState.L2.height, 0.05)
        && MathUtil.isNear(coralManipulator.getAngle(), CoralManipulatorPivotState.L2.angle, 0.25)){
            return true;
        }
        return false;
    }

    private boolean swerveAtGoal(){
        if(tooFar) return true;
        double translationTolerance = Constants.Swerve.translationTolerance;
        double rotationTolerance = Constants.Swerve.rotationTolerance;
        if(beginningState){
            translationTolerance *= 2.5;
            rotationTolerance *= 2.5;
        }
        Pose2d swervePose = swerve.getState().Pose;
        if(MathUtil.isNear(desiredPose.getX(), swervePose.getX(), translationTolerance)
        && MathUtil.isNear(desiredPose.getY(), swervePose.getY(), translationTolerance)
        && MathUtil.isNear(desiredPose.getRotation().getRadians(), swervePose.getRotation().getRadians(), rotationTolerance)){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(tooFar){
            led.blink(Color.kRed, 0.25);
        }
        else{
            led.blink(Color.kGreen, 0.25);
            coralManipulator.setPivotGoalState(CoralManipulatorPivotState.ALGAEREMOVAL);
        }
    }
}
