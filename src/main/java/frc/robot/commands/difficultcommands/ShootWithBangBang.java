package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class ShootWithBangBang extends MoveGenericSubsystem {

    private final RootNamespace namespace = new RootNamespace("shoot with bang bang");

    private static final double DEFAULT_TOLERANCE = -1.0;

    private final Shooter shooter;
    private final Supplier<Double> setpointSpeed;
    private final BangBangController bangBangController;
    private final FeedForwardSettings ffSettings = namespace.addFeedForwardNamespace("ff controller",
            FeedForwardSettings.EMPTY_FF_SETTINGS);
    private final FeedForwardController ffController;

    public ShootWithBangBang(Shooter shooter, Supplier<Double> speed, Supplier<Double> tolerance) {
        super(shooter, speed);
        this.shooter = shooter;
        this.setpointSpeed = speed;
        this.bangBangController = new BangBangController(tolerance.get());
        ffController = new FeedForwardController(ffSettings);
    }

    public ShootWithBangBang(Shooter shooter, Supplier<Double> speed) {
        this(shooter, speed, () -> DEFAULT_TOLERANCE);
    }

    @Override
    public void execute() {
        double bangBangOutput = bangBangController.calculate(shooter.getVelocity(), setpointSpeed.get());
        double ffOutput = ffController.calculate(shooter.getVelocity(), setpointSpeed.get());
        double output = ffOutput + bangBangOutput;
        shooter.move(output);
    }
}
