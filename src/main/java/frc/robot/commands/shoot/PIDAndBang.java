package frc.robot.commands.shoot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

import static java.lang.Math.abs;

public class PIDAndBang extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("shoot with pid command");

    private static final PIDSettings PID_SETTINGS = namespace.
            addPIDNamespace("shoot", new PIDSettings(0.16, 0.0005, 0.006,
                    0, 0.1, 0));

    private static final FeedForwardSettings FEED_FORWARD_SETTINGS = namespace.
            addFeedForwardNamespace("shoot", new FeedForwardSettings(0.0395, 0.1, 0,
                    FeedForwardController.ControlMode.LINEAR_VELOCITY));

    private final Shooter shooter;
    private final Supplier<Double> setpoint;
    private final BangBangController bangBangController;
    private final FeedForwardController feedForwardController;

    public PIDAndBang(Shooter shooter, Supplier<Double> setpoint, double waitTime) {
        super(shooter, new PIDSettings(PID_SETTINGS.getkP(), PID_SETTINGS.getkI(), PID_SETTINGS.getkD(),
                        PID_SETTINGS.getIZone(), PID_SETTINGS.getTolerance(), waitTime), FEED_FORWARD_SETTINGS,
                UnifiedControlMode.VELOCITY, setpoint, true);
        this.shooter = shooter;
        this.setpoint = setpoint;
        this.bangBangController = new BangBangController(PID_SETTINGS.getTolerance());
        feedForwardController = new FeedForwardController(FEED_FORWARD_SETTINGS);
    }

    @Override
    public void execute() {
        super.execute();
        if (abs(shooter.getVelocity()) <= abs(setpoint.get()) - PID_SETTINGS.getTolerance()) {
            double bangBangOutput = bangBangController.calculate(shooter.getVelocity(), setpoint.get());
            double ffOutput = feedForwardController.calculate(shooter.getVelocity(), setpoint.get());
            double output = ffOutput + bangBangOutput;
            shooter.move(output);
        }
    }
}
