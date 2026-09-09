package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFromTrench extends SequentialCommandGroup {

    private static final double SHOOT_SPEED = 3.3;

    public ShootFromTrench() {
        addCommands(
                new ShootWithPID(SHOOT_SPEED, 0.75).withTimeout(4),
                new ParallelCommandGroup(
                        new Transport(),
                        new SpinMagazine(),
                        new ShootWithPID(SHOOT_SPEED, 20) {
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
