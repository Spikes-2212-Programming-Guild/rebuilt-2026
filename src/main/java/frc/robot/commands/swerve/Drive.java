package frc.robot.commands.swerve;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.DriveSwerve;

import java.util.function.Supplier;

public class Drive extends DriveSwerve {

    public Drive(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double>
            rotationSpeed, boolean isFieldRelative, boolean useVelocityPID) {
        super(drivetrain, xSpeed, ySpeed, rotationSpeed, isFieldRelative, useVelocityPID);
    }
}
