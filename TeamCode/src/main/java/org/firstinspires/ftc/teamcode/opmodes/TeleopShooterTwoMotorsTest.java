package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterTwoMotors;
import org.firstinspires.ftc.teamcode.util.MathU;

@TeleOp(name = "Teste Shooter 2 Motores", group = "Testes")
public class TeleopShooterTwoMotorsTest extends OpMode {

    private Robot robot;
    private Drivetrain drivetrain;
    private ShooterTwoMotors shooter;
    private Intake intake;

    @Override
    public void init() {
        robot = new Robot();

        robot.init(hardwareMap);

        drivetrain = robot.getDrivetrain();
        shooter = robot.getTwoShooter();
        intake = robot.getIntake();

        telemetry.addLine("TESTE SHOOTER - 3 MOTORES");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 1:");
        telemetry.addLine("  Drive = mecanum");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 2:");
        telemetry.addLine("  LB = Par 6000 RPM + outtake configurado");
        telemetry.addLine("  LT = Par 4000 RPM + outtake configurado");
        telemetry.addLine("  RB = Intake");
        telemetry.addLine("  RT = Intake reverso");
        telemetry.addLine("  B = Shooter OFF");
        telemetry.addLine("  X = Parada geral");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.update();

        updateDrive();
        updateShooter();
        updateIntake();
        sendTelemetry();
    }

    private void updateDrive() {
        double axial;
        double lateral;
        double yaw;

        if (gamepad1.dpad_up) {
            axial = 1.0;
            lateral = 0.0;
            yaw = 0.0;
        } else if (gamepad1.dpad_down) {
            axial = -1.0;
            lateral = 0.0;
            yaw = 0.0;
        } else if (gamepad1.dpad_left) {
            axial = 0.0;
            lateral = -1.0;
            yaw = 0.0;
        } else if (gamepad1.dpad_right) {
            axial = 0.0;
            lateral = 1.0;
            yaw = 0.0;
        } else {
            axial = MathU.aplicarZonaNeutra(
                    -gamepad1.left_stick_y,
                    0.55
            );

            lateral = MathU.aplicarZonaNeutra(
                    gamepad1.left_stick_x,
                    0.55
            ) * 1.1;

            yaw = MathU.aplicarZonaNeutra(
                    gamepad1.right_stick_x,
                    0.55
            );
        }

        double speedMultiplier = gamepad1.left_bumper
                ? DrivetrainConfig.DRIVE_POWER_NORMAL
                : DrivetrainConfig.DRIVE_POWER_TURBO;

        drivetrain.driveMecanum(
                axial,
                lateral,
                yaw,
                speedMultiplier
        );
    }

    private void updateShooter() {
        if (shooter == null) {
            return;
        }

        if (gamepad2.x) {
            shooter.stop();
            return;
        }

        if (gamepad2.b) {
            shooter.stop();
            return;
        }

        if (gamepad2.left_bumper) {
            shooter.setRPM(ShooterConfig.MANUAL_SHOOTER_RPM, ShooterConfig.DEFAULT_OUTTAKE_RPM);
            return;
        }

        if (gamepad2.left_trigger
                > RConstants.GAMEPAD_TRIGGER_THRESHOLD) {

            shooter.setRPM(4000.0, ShooterConfig.DEFAULT_OUTTAKE_RPM);
            return;
        }

        shooter.stop();
    }

    private void updateIntake() {
        if (intake == null) {
            return;
        }

        if (gamepad2.x) {
            intake.stop();
            return;
        }

        if (gamepad2.right_trigger
                > RConstants.GAMEPAD_TRIGGER_THRESHOLD) {

            intake.startReverseContinuous();
        } else if (gamepad2.right_bumper) {

            intake.startContinuous();
        } else {

            intake.stopContinuous();
        }
    }

    private void sendTelemetry() {
        telemetry.addLine("===== SHOOTER 3 MOTORES =====");

        if (shooter == null) {
            telemetry.addLine("ShooterTwoMotors = NULL");
        } else {
            telemetry.addData(
                    "RPM alvo",
                    shooter.getTargetRPM()
            );

            telemetry.addData(
                    "RPM esquerdo",
                    shooter.getLeftRPM()
            );

            telemetry.addData(
                    "RPM direito",
                    shooter.getRightRPM()
            );

            telemetry.addData(
                    "RPM médio paralelo",
                    shooter.getCurrentRPM()
            );

            telemetry.addData(
                    "RPM outtake",
                    shooter.getOuttakeRPM()
            );

            telemetry.addData(
                    "RPM alvo outtake",
                    shooter.getTargetOuttakeRPM()
            );

            telemetry.addData(
                    "RPM motor outtake",
                    shooter.getOuttakeMotorRPM()
            );

            telemetry.addData(
                    "Outtake ticks/s",
                    shooter.getOuttakeTicksPerSecond()
            );

            telemetry.addData(
                    "Encoder outtake",
                    shooter.getOuttakeEncoderPosition()
            );

            telemetry.addData(
                    "Motor esquerdo RPM",
                    shooter.getLeftMotorRPM()
            );

            telemetry.addData(
                    "Motor direito RPM",
                    shooter.getRightMotorRPM()
            );

            telemetry.addData(
                    "Esquerdo ticks/s",
                    shooter.getLeftTicksPerSecond()
            );

            telemetry.addData(
                    "Direito ticks/s",
                    shooter.getRightTicksPerSecond()
            );

            telemetry.addData(
                    "Encoder esquerdo",
                    shooter.getLeftEncoderPosition()
            );

            telemetry.addData(
                    "Encoder direito",
                    shooter.getRightEncoderPosition()
            );

            telemetry.addData(
                    "Shooter pronto",
                    shooter.isAtTargetRPM()
            );
        }

        telemetry.addLine("");
        telemetry.addLine("===== CONTROLES =====");
        telemetry.addData("LB", "Par 6000 RPM / outtake padrão");
        telemetry.addData("LT", "Par 4000 RPM / outtake padrão");
        telemetry.addData("B", "Shooter OFF");
        telemetry.addData("X", "PARADA GERAL");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAll();
            robot.closeVision();
        }
    }
}