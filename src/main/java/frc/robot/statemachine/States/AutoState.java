package frc.robot.statemachine.States;

import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AutoEnums.AutoSteps;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;

public class AutoState extends State {
  public AutoState(StateMachineBase stateMachine, LoggedDashboardChooser<ArrayList<AutoSteps>> autoChooser,
      Turret turret, Hood hood, Intake intake, Spindexer spindexer) {
    super(stateMachine);

    // t(() -> turret.inHoodDangerZone() &&
    // !hood.calibrated()).whileTrue(turret.exitDangerZone());
    startWhenActive(intake.intakeBottom());
    startWhenActive(intake.runRollers());

    startWhenActive(spindexer.setSpeed());
  }
}
