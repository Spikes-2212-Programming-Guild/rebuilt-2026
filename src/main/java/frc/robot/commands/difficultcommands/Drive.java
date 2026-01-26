package frc.robot.commands.difficultcommands;

import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.com.spikes2212.command.drivetrains.swerve.commands.DriveSwerve;

import java.util.function.Supplier;

public class Drive extends DriveSwerve {

    //@TODO add a stick drift limit
    public Drive(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double>
            rotationSpeed, boolean isFieldRelative, boolean useVelocityPID) {
        super(drivetrain, xSpeed, ySpeed, rotationSpeed, isFieldRelative, useVelocityPID);
    }
}
