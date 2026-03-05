package frc.robot.commands.advancedcommand;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.difficultcommands.RotateHood;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.subsystems.*;
import frc.robot.utils.ShooterAlgo;
import java.util.function.Supplier;

public class ShootingWithRegression extends SequentialCommandGroup {

    public ShootingWithRegression(Hood hood, Shooter shooter, SpinningMagazine spinningMagazine,
                                  Transport transport, Supplier<Double> distance) {
        addCommands(
                Commands.deferredProxy(() -> {
                    Constants.ShooterConstants.FlywheelQuadratic targetPose =
                            ShooterAlgo.getOptimalPose(distance.get());
                    double targetRPM = ShooterAlgo.calculateRPM(distance.get(), targetPose);

                    Hood.HoodPose hoodPose = switch (targetPose) {
                        case SHOOT_POSE1 -> Hood.HoodPose.POSE1;
                        case SHOOT_POSE2 -> Hood.HoodPose.POSE2;
                        case SHOOT_POSE3 -> Hood.HoodPose.POSE3;
                    };

                    return Commands.sequence(
                            new ParallelCommandGroup(
                                    new RotateHood(hood, hoodPose),
                                    new ShootWithPID(shooter, () -> targetRPM)
                            ),

                            new ParallelCommandGroup(
                                    new RunCommand(() -> spinningMagazine.move(SpinningMagazine.SPEED), spinningMagazine),
                                    new RunCommand(() -> transport.move(Transport.SPEED), transport)
                            ).withTimeout(1.0)

                    ).finallyDo((interrupted) -> {
                        shooter.stop();
                        spinningMagazine.stop();
                        transport.stop();
                    });
                }));
    }
}
