package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.RotateHood;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.commands.simplecommands.SimpleTransport;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpinningMagazine;
import frc.robot.subsystems.Transport;

import java.util.function.Supplier;

public class Pass extends SequentialCommandGroup {

    public Pass(Hood hood, Shooter shooter, Supplier<Double> shootingSpeed,
                SpinningMagazine spinningMagazine,
                Transport transport
                ) {
        addCommands(
                new ParallelDeadlineGroup(
                        new RotateHood(hood, Hood.HoodPose.PASS_ANGLE),
                        new SimpleShoot(shooter, shootingSpeed)
                ),
                new ParallelCommandGroup(
                        new SimpleSpin(spinningMagazine),
                        new SimpleTransport(transport),
                        new SimpleShoot(shooter, shootingSpeed)
                )
        );
    }
}
