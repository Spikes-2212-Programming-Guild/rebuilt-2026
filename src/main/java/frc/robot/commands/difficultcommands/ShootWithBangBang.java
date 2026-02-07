package frc.robot.commands.difficultcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class ShootWithBangBang extends MoveGenericSubsystem {

    private static final double DEFAULT_TOLERANCE = -1.0;

    private final Shooter shooter;
    private final Supplier<Double> setpointSpeed;
    private final BangBangController bangBangController;

    public ShootWithBangBang(Shooter shooter, Supplier<Double> speed, Supplier<Double> tolerance) {
        super(shooter, speed);
        this.shooter = shooter;
        this.setpointSpeed = speed;
        this.bangBangController = new BangBangController(tolerance.get());
    }

    public ShootWithBangBang(Shooter shooter, Supplier<Double> speed) {
        this(shooter, speed, () -> DEFAULT_TOLERANCE);
    }

    @Override
    public void execute() {
        double output = bangBangController.calculate(shooter.getVelocity(), setpointSpeed.get());
        shooter.setSpeed(output);
    }
}
