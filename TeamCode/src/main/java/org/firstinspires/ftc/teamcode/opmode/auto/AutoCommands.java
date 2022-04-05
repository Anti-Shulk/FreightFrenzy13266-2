package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class AutoCommands {
    LinearOpMode opMode;
    BooleanSupplier isStopRequested;
    public AutoCommands(LinearOpMode opMode) {
        this.opMode = opMode;
        this.isStopRequested = opMode::isStopRequested;
    }
    private boolean isStopRequested() {
        return isStopRequested.getAsBoolean();
    }

//    public MarkerCallback runCommandGroupAsThreadInTrajectorySequence(SequentialCommandGroup sequentialCommandGroup) {
//
//        return () -> new Thread(() -> {
//            if (!isStopRequested()) sequentialCommandGroup.initialize();
//
//            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
//                sequentialCommandGroup.execute();
//            }
//        }).start();
//    }
//
//    public MarkerCallback runCommandGroupAsThreadInTrajectorySequence(SequentialCommandGroup sequentialCommandGroup, double seconds, Runnable stopCommand) {
//
//        return () -> new Thread(() -> {
//            ElapsedTime elapsedTime = new ElapsedTime();
//            double targetTime = elapsedTime.seconds() + seconds;
//            while (targetTime > elapsedTime.seconds()) {
//                if (!isStopRequested()) sequentialCommandGroup.initialize();
//
//                while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
//                    sequentialCommandGroup.execute();
//                }
//            }
//            stopCommand.run();
//        }).start();
//    }
    public void runCommandGroupAsThread(SequentialCommandGroup sequentialCommandGroup) {
        new Thread(() -> {
            if (!isStopRequested()) sequentialCommandGroup.initialize();

            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                sequentialCommandGroup.execute();
            }
        }).start();
    }
    public void runCommandGroup(CommandBase command) {
            if (!isStopRequested()) command.initialize();

            while (!isStopRequested() && !command.isFinished()) {
                command.execute();
            }
            command.end(false);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
//    public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup, double seconds, Runnable stopCommand) {
//        new Thread(() -> {
//            ElapsedTime elapsedTime = new ElapsedTime();
//            double targetTime = elapsedTime.seconds() + seconds;
//            while (targetTime > elapsedTime.seconds()) {
//                if (!isStopRequested()) sequentialCommandGroup.initialize();
//
//                while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
//                    sequentialCommandGroup.execute();
//                }
//            }
//            stopCommand.run();
//        }).start();
//    }

}
