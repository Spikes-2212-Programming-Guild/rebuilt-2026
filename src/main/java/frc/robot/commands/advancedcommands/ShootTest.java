package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class ShootTest extends SequentialCommandGroup {

    private static final Supplier<Double> speed = Robot.speed;

    public ShootTest() {
        addCommands(
                new ShootWithPID(speed, 0.75).withTimeout(4),
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(),
                        new ShootWithPID(speed, 20) {
                            @Override
                            public void end(boolean interrupted) {
                                Shooter.getInstance().stop();
                            }

                            @Override
                            public boolean isFinished() {
                                return false;
                            }
                        }
                )
        );
    }
}
