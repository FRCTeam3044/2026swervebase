package frc.robot.statemachine.States;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class TeleState extends State {
  public TeleState(StateMachineBase stateMachine, Drive drive) {
    super(stateMachine);
    startwhenActive(DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));
  }
}
