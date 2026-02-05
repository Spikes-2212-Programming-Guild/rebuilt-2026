package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class DifficultShootWithBangBang extends MoveSmartMotorControllerGenericSubsystem {

    private static final Shooter shooter = Shooter.getInstance();

    private static final RootNamespace namespace = new RootNamespace("difficult shoot - bang bang controller");

    private static final FeedForwardSettings ffSettings = namespace.
            addFeedForwardNamespace("difficult shoot - ff", FeedForwardSettings.EMPTY_FF_SETTINGS);

    private static final PIDSettings pidSettings = PIDSettings.EMPTY_PID_SETTINGS;

    private static BangBangController bangBangController = new BangBangController();

    private final Supplier<Double> speed;

    public DifficultShootWithBangBang(Shooter shooter, FeedForwardSettings feedForwardSettings, Supplier<Double> speed,
                                      Supplier<Double> tolorence) {
        super(shooter, pidSettings, feedForwardSettings, UnifiedControlMode.VELOCITY, speed,true);
        this.speed = speed;
        bangBangController = new BangBangController(tolorence.get());
    }

    public DifficultShootWithBangBang(Shooter shooter, FeedForwardSettings feedForwardSettings, Supplier<Double> speed) {
        super(shooter, pidSettings, feedForwardSettings, UnifiedControlMode.VELOCITY, speed,true);
        this.speed = speed;
    }

    @Override
    public void execute() {
        super.execute();
        shooter.setSpeed(bangBangController.calculate(shooter.getSpeed(), speed.get()));
    }


}
