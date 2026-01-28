package frc.robot.statemachine;

import frc.robot.statemachine.States.ActiveHub;
import frc.robot.statemachine.States.AlliedZone;
import frc.robot.statemachine.States.InactiveHub;
import frc.robot.statemachine.States.NeutralZone;
import me.nabdev.oxidation.StateMachineBase;

public class StateMachine extends StateMachineBase {
    public StateMachine() {
        super();

        NeutralZone neutralZone = new NeutralZone(this);
        AlliedZone alliedZone = new AlliedZone(this);
        ActiveHub activeHub = new ActiveHub(this);
        InactiveHub inactiveHub = new InactiveHub(this);

        alliedZone.withChild(inactiveHub, () -> false, 0, "Inactive Hub");
        alliedZone.withChild(activeHub, () -> false, 1, "Active Hub");

        alliedZone.withTransition(neutralZone, () -> false, 0, "Drive into Neutral Zone");
        neutralZone.withTransition(alliedZone, () -> false, 0, "Drive into Allied Zone");

        activeHub.withTransition(inactiveHub, () -> false, 0, "Hub becomes inactive");
        inactiveHub.withTransition(activeHub, () -> false, 0, "Hub becomes active");
    }
}
