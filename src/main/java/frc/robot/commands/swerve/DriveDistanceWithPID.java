package frc.robot.commands.swerve;

import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveDistanceWithPID extends Command {

    public enum DriveDirection {
        X, Y;
    }

    private static final RootNamespace namespace = new RootNamespace("drive distance");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("drive");

    private static final double TIME_STEP = 0.02;

    private final Drivetrain drivetrain;
    private final double targetPosition;
    private final DriveDirection driveDirection;
    private final boolean updatePidPeriodically;
    private final PIDController pidController;

    private double lastTimeNotOnTarget;

    /*
        note - the drive is field relative, so it could use the gyro to maybe correct drive angle,
        and make it easier to program, but it is subject to change
     */
    public DriveDistanceWithPID(Drivetrain drivetrain, double distance,
                                DriveDirection driveDirection, boolean updatePidPeriodically) {
        this.drivetrain = drivetrain;
        this.targetPosition = getPosition() + distance; // could reset the module encoder instead
        this.driveDirection = driveDirection;
        this.updatePidPeriodically = updatePidPeriodically;

        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());

        lastTimeNotOnTarget = 0;
        configureDashboard();
    }

    @Override
    public void execute() {
        namespace.update();
        if (updatePidPeriodically) {
            updatePidGains();
        }

        double position = getPosition();
        double speed = pidController.calculate(position, targetPosition);

        if (driveDirection.equals(DriveDirection.X)) {
            drivetrain.drive(speed, 0, 0, false, TIME_STEP, true);
        } else {
            drivetrain.drive(0, speed, 0, false, TIME_STEP, true);
        }
    }


    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    private void updatePidGains() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());
    }

    private double getPosition() {
        return this.drivetrain.getBackLeftModule().getModulePosition().distanceMeters;
    }

    private void configureDashboard() {
        namespace.putNumber("current position", this::getPosition);
        namespace.putNumber("target position", () -> targetPosition);
    }
}
