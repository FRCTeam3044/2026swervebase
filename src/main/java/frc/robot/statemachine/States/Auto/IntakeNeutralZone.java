package frc.robot.statemachine.States.Auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class IntakeNeutralZone extends State {
  public IntakeNeutralZone(StateMachineBase stateMachine, Supplier<ArrayList<Pose2d>> waypoints, Drive drive,
      Intake intake) {
    super(stateMachine);

    startWhenActive(
        DriveCommands.goToPoints(drive, waypoints, () -> Rotation2d.fromDegrees(45)).withName("second pathfind"));
  }
}
