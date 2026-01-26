package frc.robot.commands.difficultCommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Hood;

import java.util.function.Supplier;

public class RotateHood extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("rotate hood");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("pid settings");
    private static final FeedForwardSettings feedForwardSettings =
            namespace.addFeedForwardNamespace("ff settings", FeedForwardController.ControlMode.LINEAR_POSITION);

    private final Hood hood;

    public RotateHood(Hood hood, Supplier<Double> setpoint) {
        super(hood, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION,
                setpoint, true);
        this.hood = hood;
    }

    public RotateHood(Hood hood, Hood.HoodPose pose) {
        this(hood, () -> pose.neededAngle);
    }

    @Override
    public boolean isFinished() {
        return !(hood.canMove(hood.getSpeed())) || super.isFinished();
    }
}
