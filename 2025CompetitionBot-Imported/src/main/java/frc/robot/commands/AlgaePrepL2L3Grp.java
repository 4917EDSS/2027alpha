package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaePrepL2L3Grp extends SequentialCommandGroup {
  /** Creates a new AlgaeRemovalL2L3Grp. */
  public AlgaePrepL2L3Grp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveElArmGrp(Constants.Elevator.kCoralGrabbableHeight, Constants.Arm.kL2L3AlgaeRemovalPrepAngle, armSub,
            elevatorSub), // Move elevator and arm to algae removal location
        new MoveElArmGrp(Constants.Elevator.kL2L3AlgaeRemovalPrepHeight, Constants.Arm.kL2L3AlgaeRemovalPrepAngle,
            armSub, elevatorSub));
  }
}
