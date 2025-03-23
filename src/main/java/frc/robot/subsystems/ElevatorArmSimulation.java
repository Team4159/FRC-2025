// package frc.robot.subsystems;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ElevatorArmSimulation extends SubsystemBase{
//     private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
//     private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("CoralSimulator", 10, 0);
//     private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
//         new MechanismLigament2d("Elevator", 10, 90));
//     private final MechanismLigament2d m_CoralManipulatorPivotMech2d = m_elevatorMech2d.append(
//         new MechanismLigament2d("CoralManipulatorPivot", 10, 0));

//     private final Elevator elevator;
//     private final CoralManipulatorPivot ;

//     public ElevatorArmSimulation(Elevator elevator, CoralManipulatorPivot ){
//         this.elevator = elevator;
//         this. = ;
//         SmartDashboard.putData("CoralSimulator", m_mech2d);
//     }

//     @Override
//     public void periodic(){
//         m_elevatorMech2d.setLength(elevator.getSimPosition() * 50 + 5);
//         m_CoralManipulatorPivotMech2d.setAngle(Units.radiansToDegrees(.getSimPosition()));
//     }
// }
