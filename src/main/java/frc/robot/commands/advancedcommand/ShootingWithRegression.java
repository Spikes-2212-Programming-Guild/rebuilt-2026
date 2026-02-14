package frc.robot.commands.advancedcommand;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.RotateHood;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.commands.simplecommands.SimpleTransport;
import frc.robot.subsystems.*;
import frc.robot.utils.ShooterAlgo;
import java.util.function.Supplier;

public class ShootingWithRegression extends SequentialCommandGroup {

    public ShootingWithRegression(Hood hood, Shooter shooter, SpinningMagazine spinningMagazine,
                                  Transport transport, Supplier<Double> distance) {
        addCommands(
                Commands.deferredProxy(() -> {
                    var targetPose = ShooterAlgo.getOptimalPose(distance.get());
                    double targetRPM = ShooterAlgo.calculateRPM(distance.get(), targetPose);
                    Hood.HoodPose hoodPose = null;
                    switch (targetPose) {
                        case SHOOT_POSE1:
                            hoodPose = Hood.HoodPose.POSE1;
                        case SHOOT_POSE2:
                            hoodPose = Hood.HoodPose.POSE2;
                        case SHOOT_POSE3:
                            hoodPose = Hood.HoodPose.POSE3;
                    }


                    return Commands.sequence(
                            new ParallelCommandGroup(
                                    new RotateHood(hood, hoodPose),
                                    new ShootWithPID(shooter, () -> targetRPM)
                            ),
                            new ParallelCommandGroup(
                                    new SimpleSpin(spinningMagazine),
                                    new SimpleTransport(transport)
                            ),
                            new ParallelCommandGroup(
                                    Commands.runOnce(shooter::stop, shooter),
                                    Commands.runOnce(spinningMagazine::stop, spinningMagazine),
                                    Commands.runOnce(transport::stop, transport)
                            )
                    );
                }));
    }
}
