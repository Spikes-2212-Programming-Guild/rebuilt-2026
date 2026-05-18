package frc.robot.oi;

import com.spikes2212.dashboard.ChildNamespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.oi.devices.Controller;
import frc.robot.oi.devices.DoubleJoystick;
import frc.robot.oi.devices.InputDevice;
import frc.robot.oi.devices.SingleJoystick;

import java.util.function.Supplier;

/*
an overkill system that lets you switch between different
input devices in runtime and toggle between different control modes

to use:
set the ports in the dashboard
and run "Switch to" the device

you can view the current device name and control mode on the dashboard
and toggle between them
 */
public class OI {

    private final ChildNamespace namespace;
    private InputDevice device = null;

    private final Actions actions = new Actions(
            this::toggleSpeedScale,
            this::toggleFieldRelative,
            this::toggleSquareInputs,
            this::toggleDeadband,
            this::toggleSlewRateLimiter
    );

    // TODO - tune these
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(3);

    // TODO - maybe add these to the dashboard
    private static final double LOW_SPEED = 0.7;
    private static final double HIGH_SPEED = 1;
    private static final double DEADBAND = 0.05;

    private double speedScale = LOW_SPEED;
    private boolean useFieldRelative = false;
    private boolean useSquareInputs = false;
    private boolean useSlewRateLimiter = false;
    private boolean useDeadband = false;

    public OI(RootNamespace rootNamespace) {
        this.namespace = rootNamespace.addChild("oi");
        setupActions();
        setupDevices();
    }

    private void setupActions() {
        namespace.putRunnable("toggle speed scale", this::toggleSpeedScale);
        namespace.putRunnable("toggle field relative", this::toggleFieldRelative);
        namespace.putRunnable("toggle square inputs", this::toggleSquareInputs);
        namespace.putRunnable("toggle deadband", this::toggleDeadband);
        namespace.putRunnable("toggle slew rate limiter", this::toggleSlewRateLimiter);

        namespace.putNumber("speed scale", () -> speedScale);
        namespace.putBoolean("is field relative", () -> useFieldRelative);
        namespace.putBoolean("is square inputs", () -> useSquareInputs);
        namespace.putBoolean("is slew rate limiter", () -> useSlewRateLimiter);
        namespace.putBoolean("is deadband", () -> useDeadband);

        namespace.putString("current device", () -> {
            if (device != null) return device.getName();
            return "none";
        });
    }

    private void setupDevices() {
        Supplier<Integer> controllerPort = namespace.addConstantInt("controller port", 0);
        namespace.putRunnable("switch to controller", () -> setDevice(
                new Controller(controllerPort.get())
        ));

        Supplier<Integer> singleJoystickPort = namespace.addConstantInt("single joystick port", 0);
        namespace.putRunnable("switch to single joystick", () -> setDevice(
                new SingleJoystick(singleJoystickPort.get())
        ));

        Supplier<Integer> doubleJoystickLeftPort = namespace.addConstantInt("double joystick left port", 0);
        Supplier<Integer> doubleJoystickRightPort = namespace.addConstantInt("double joystick right port", 0);
        namespace.putRunnable("switch to double joystick", () -> setDevice(new DoubleJoystick(
                doubleJoystickLeftPort.get(), doubleJoystickRightPort.get())
        ));
    }

    public double getX() {
        if (device == null) return 0;
        return calculateInput(device.getX(), xLimiter);
    }

    public double getY() {
        if (device == null) return 0;
        return calculateInput(device.getY(), yLimiter);
    }

    public double getZ() {
        if (device == null) return 0;
        return calculateInput(device.getZ(), zLimiter);
    }

    public boolean useFieldRelative() {
        return this.useFieldRelative;
    }

    private void setDevice(InputDevice device) {
        this.device = device;
        this.device.bindActions(actions);
    }

    private double calculateInput(double value, SlewRateLimiter limiter) {
        if (useDeadband) value = MathUtil.applyDeadband(value, DEADBAND);
        if (useSquareInputs) value = Math.copySign(value * value, value);
        value *= speedScale;
        if (useSlewRateLimiter) value = limiter.calculate(value);
        return value;
    }

    private void toggleSpeedScale() {
        if (speedScale == LOW_SPEED) {
            speedScale = HIGH_SPEED;
        } else {
            speedScale = LOW_SPEED;
        }
    }

    private void toggleFieldRelative() {
        this.useFieldRelative = !useFieldRelative;
    }

    private void toggleDeadband() {
        this.useDeadband = !useDeadband;
    }

    private void toggleSlewRateLimiter() {
        this.useSlewRateLimiter = !useSlewRateLimiter;
    }

    private void toggleSquareInputs() {
        this.useSquareInputs = !useSquareInputs;
    }
}
