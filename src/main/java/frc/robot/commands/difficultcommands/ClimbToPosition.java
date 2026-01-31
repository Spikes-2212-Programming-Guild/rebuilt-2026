package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Climb;

import java.util.function.Supplier;

public class ClimbToPosition extends MoveGenericSubsystem {

    public ClimbToPosition(Climb climb, Climb.ArmPose armPose) {
        super(climb, armPose::getNeededSpeed);
    }

    public ClimbToPosition(Climb climb, Supplier<Double> speed) {
        super(climb, speed);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || !subsystem.canMove(subsystem.getSpeed());
    }
}
