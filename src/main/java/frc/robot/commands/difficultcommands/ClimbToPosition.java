package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Climb;

import java.util.function.Supplier;

public class ClimbToPosition extends MoveGenericSubsystem {

    public enum ClimbPosition {

        TOP(-1), BOTTOM(-1);

        private final double neededSpeed;

        public double getNeededSpeed() {
            return neededSpeed;
        }

        ClimbPosition(double neededSpeed) {
            this.neededSpeed = neededSpeed;
        }
    }

    public ClimbToPosition(Climb climb, ClimbPosition climbPosition) {
        super(climb, climbPosition::getNeededSpeed);
    }

    public ClimbToPosition(Climb climb, Supplier<Double> speed) {
        super(climb, speed);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || !subsystem.canMove(subsystem.getSpeed());
    }
}
