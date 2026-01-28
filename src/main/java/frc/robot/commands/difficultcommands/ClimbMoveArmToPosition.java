package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Climb;

import java.util.function.Supplier;

import static com.spikes2212.control.FeedForwardController.ControlMode.LINEAR_POSITION;

public class ClimbMoveArmToPosition extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("arm to position");

    private static final PIDSettings pidSettings = namespace.addPIDNamespace("arm to position");
    private static final FeedForwardSettings ffSettings = namespace.
            addFeedForwardNamespace("arm to position", LINEAR_POSITION);

    public ClimbMoveArmToPosition(Climb climb, Climb.ArmPose armPose) {
        super(climb, pidSettings, ffSettings, UnifiedControlMode.POSITION,
                armPose::getPositionMeters, true);
    }

    public ClimbMoveArmToPosition(Climb climb, Supplier<Double> positionMeters) {
        super(climb, pidSettings, ffSettings, UnifiedControlMode.POSITION, positionMeters, true);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || !subsystem.canMove(subsystem.getSpeed());
    }
}
