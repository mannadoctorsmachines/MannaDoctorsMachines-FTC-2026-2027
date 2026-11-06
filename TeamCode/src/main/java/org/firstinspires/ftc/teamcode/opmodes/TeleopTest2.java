package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.seasons.biobuzz.BallRetainer;
import org.firstinspires.ftc.teamcode.seasons.biobuzz.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterTwoMotors;
import org.firstinspires.ftc.teamcode.vision.VisionConfig;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.util.CalcDistAlvo;
import org.firstinspires.ftc.teamcode.util.MathU;
import org.firstinspires.ftc.teamcode.util.ShooterLista;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp(name = "Teste do teste", group = "Competition")
public class TeleopTest2 extends OpMode {

    private enum ShooterMode {
        OFF,
        MANUAL,
        REVERSE,
        AUTO
    }

    private Robot robot;
    private BiobuzzRobot biobuzzRobot;

    private Drivetrain drive;
    private Vision vision;

    /*
     * Shooter principal.
     *
     * O ShooterTwoMotors suporta:
     * - 2 motores: usa os dois;
     * - 1 motor: usa somente o motor esquerdo.
     */
    private ShooterTwoMotors shooterTwoMotors;

    private Intake intake;
    private BallRetainer ballRetainer;

    private CalcDistAlvo distanceCalculator;
    private ShooterLista shooterTable;

    private double lastRawCameraDistanceCm = 0.0;
    private double lastUnfilteredShooterDistanceCm = 0.0;
    private double lastShooterDistanceCm = 0.0;
    private double lastTargetRPM = ShooterConfig.DEFAULT_SHOOTER_RPM;

    private boolean hasCameraShooterSolution = false;
    private long lastCameraSolutionTimeMs = 0;
    private int lastTargetId = -1;

    private boolean shooterSequenceActive = false;
    private long shooterStartTimeMs = 0;

    private ShooterMode shooterMode = ShooterMode.OFF;

    private double activeTargetRPM = 0.0;

    private boolean autoShooterWaitingForCamera = false;

    private boolean intakeRunning = false;
    private boolean intakeReversing = false;
    private boolean intakeBlockedByShooter = false;

    /*
     * O Intake só espera quando o shooter está em AUTO (LT).
     *
     * LB = manual: Intake pode funcionar imediatamente.
     * LT = automático: Intake espera 3 segundos.
     */
    private static final long AUTO_SHOOTER_INTAKE_DELAY_MS = 3000;

    private boolean autoIntakeDelayActive = false;
    private long autoIntakeDelayRemainingMs = 0;

    private boolean alignmentRequested = false;
    private boolean alignedWithTag = false;

    @Override
    public void init() {

        biobuzzRobot = new BiobuzzRobot();
        biobuzzRobot.init(hardwareMap);

        robot = biobuzzRobot.getRobot();
        robot.initVision(hardwareMap);

        drive = robot.getDrivetrain();

        shooterTwoMotors = robot.getTwoShooter();

        intake = robot.getIntake();
        vision = robot.getVision();

        ballRetainer = biobuzzRobot.getBallRetainer();

        distanceCalculator = new CalcDistAlvo();
        shooterTable = new ShooterLista();

        telemetry.addLine("DECODE TeleOp pronto.");
        telemetry.addLine("gamepad2 LB: shooter manual");
        telemetry.addLine("gamepad2 LT: autoshooter pela câmera");
        telemetry.addLine("gamepad2 Y: shooter reverso");
        telemetry.addLine("gamepad2 RB: intake para frente");
        telemetry.addLine("gamepad2 RT: intake reverso/destravar");
        telemetry.addLine("gamepad2 X: parada dos mecanismos/alinhamento");
        telemetry.addLine("gamepad2 B: parada dos mecanismos");
        telemetry.addLine("gamepad2 A: Trava do Outtake");
        telemetry.update();
    }

    @Override
    public void loop() {

        biobuzzRobot.update();
        ballRetainer.update();

        updateCameraAndShooterSolution();
        updateDrive();
        updateMechanisms();

        sendTelemetry();

        updateBallRetainer();
    }

    /**
     * No TeleOp, a webcam Logitech mede a distância pela AprilTag.
     * A tabela converte essa distância no RPM necessário para o shooter.
     */
    private void updateCameraAndShooterSolution() {

        if (vision == null) {
            return;
        }

        /*
         * robot.update() já atualizou a visão neste ciclo.
         */
        if (!vision.hasTarget() || !vision.hasRange()) {
            return;
        }

        int currentTargetId = vision.getTargetId();

        boolean solutionExpired = !hasFreshCameraShooterSolution();
        boolean targetChanged = lastTargetId >= 0 && currentTargetId != lastTargetId;

        if (solutionExpired || targetChanged) {
            distanceCalculator.resetDistanceFilter();
        }

        double rangeInches = vision.getRangeInches();

        lastRawCameraDistanceCm =
                distanceCalculator.getCameraDistanceCm(rangeInches);

        lastUnfilteredShooterDistanceCm =
                distanceCalculator.getShooterDistanceCm(rangeInches);

        lastShooterDistanceCm =
                distanceCalculator.updateFilteredShooterDistanceCm(rangeInches);

        lastTargetRPM = shooterTable.getRPMForDistance(lastShooterDistanceCm);

        hasCameraShooterSolution = true;
        lastCameraSolutionTimeMs = System.currentTimeMillis();
        lastTargetId = currentTargetId;
    }

    private boolean hasFreshCameraShooterSolution() {

        return hasCameraShooterSolution
                && System.currentTimeMillis() - lastCameraSolutionTimeMs
                <= VisionConfig.CAMERA_SOLUTION_MAX_AGE_MS;
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

            axial = MathU.aplicarZonaNeutra(-gamepad1.left_stick_y, 0.55);
            lateral = MathU.aplicarZonaNeutra(gamepad1.left_stick_x, 0.55) * 1.1;
            yaw = MathU.aplicarZonaNeutra(gamepad1.right_stick_x, 0.55);
        }

        double speedMultiplier = gamepad1.left_bumper ? DrivetrainConfig.DRIVE_POWER_NORMAL
                : DrivetrainConfig.DRIVE_POWER_TURBO;

        alignmentRequested = gamepad2.x;
        alignedWithTag = false;

        if (alignmentRequested) {

            /*
             * O botão X só pode girar o robô.
             * Nenhuma potência de translação é enviada
             * enquanto o alinhamento estiver ativo.
             */
            axial = 0.0;
            lateral = 0.0;
            yaw = 0.0;

            if (vision != null && vision.hasTarget()) {

                double bearingDegrees = vision.getBearingDegrees();

                alignedWithTag = Math.abs(bearingDegrees) <= VisionConfig.AIM_TOLERANCE_DEGREES;

                if (!alignedWithTag) {
                    double targetHeadingDegrees = drive.getHeadingDegrees() + bearingDegrees;
                    yaw = drive.getTurnPowerToHeading(targetHeadingDegrees);
                }
            }
        }

        drive.driveMecanum(axial, lateral, yaw, speedMultiplier);
    }

    private void updateMechanisms() {

        /*
         * Controles:
         *
         * LB = Shooter manual
         * LT = Shooter automático pela câmera
         * RB = Intake para frente
         * RT = Intake reverso
         * X  = parada de emergência
         */

        boolean emergencyStop = gamepad2.x;
        boolean reverseShooterRequested = gamepad2.y;
        boolean manualShooterRequested = gamepad2.left_bumper;
        boolean autoShooterRequested = gamepad2.left_trigger > RConstants.GAMEPAD_TRIGGER_THRESHOLD;

        boolean forwardIntakeRequested = gamepad2.right_bumper;
        boolean reverseIntakeRequested = gamepad2.right_trigger > RConstants.GAMEPAD_TRIGGER_THRESHOLD;

        if (emergencyStop) {

            shooterSequenceActive = false;
            shooterMode = ShooterMode.OFF;
            activeTargetRPM = 0.0;
            autoShooterWaitingForCamera = false;

            autoIntakeDelayActive = false;
            autoIntakeDelayRemainingMs = 0;

            if (shooterTwoMotors != null) {
                shooterTwoMotors.stop();
            }

            if (intake != null) {
                intake.stop();
            }

            intakeRunning = false;
            intakeReversing = false;
            intakeBlockedByShooter = false;

            return;
        }

        boolean hasAutoSolution = hasFreshCameraShooterSolution();

        autoShooterWaitingForCamera = autoShooterRequested && !hasAutoSolution;

        ShooterMode requestedShooterMode = ShooterMode.OFF;
        /*
         * O AUTO tem prioridade quando LT e LB
         * forem pressionados juntos.
         *
         * Se LT estiver sem alvo, LB ainda pode
         * assumir o modo manual.
         */

        if (reverseShooterRequested){

            requestedShooterMode = ShooterMode.REVERSE;
        } else if (autoShooterRequested) {

            requestedShooterMode = ShooterMode.AUTO;

        } else if (manualShooterRequested) {

            requestedShooterMode = ShooterMode.MANUAL;
        }

        updateShooter(requestedShooterMode);

        long shooterElapsedMs =
                shooterSequenceActive
                        ? System.currentTimeMillis() - shooterStartTimeMs
                        : 0;

        /*
         * =====================================================
         * INTAKE
         * =====================================================
         *
         * LB:
         *     Intake funciona imediatamente.
         *
         * LT:
         *     Intake espera 3 segundos.
         *
         * RT:
         *     Reverso funciona imediatamente.
         */

        intakeReversing = reverseIntakeRequested;

        autoIntakeDelayActive =
                shooterMode == ShooterMode.AUTO && shooterSequenceActive
                        && shooterElapsedMs < AUTO_SHOOTER_INTAKE_DELAY_MS;

        if (autoIntakeDelayActive) {

            autoIntakeDelayRemainingMs = AUTO_SHOOTER_INTAKE_DELAY_MS - shooterElapsedMs;

        } else {

            autoIntakeDelayRemainingMs = 0;
        }

        /*
         * RT sempre tem prioridade para permitir
         * destravar o Intake imediatamente.
         */
        if (reverseIntakeRequested) {

            intakeRunning = true;
            intakeBlockedByShooter = false;

        } else if (forwardIntakeRequested) {

            if (autoIntakeDelayActive) {

                intakeRunning = false;
                intakeBlockedByShooter = true;

            } else {

                intakeRunning = true;
                intakeBlockedByShooter = false;
            }

        } else {

            intakeRunning = false;
            intakeBlockedByShooter = false;
        }

        if (intake != null) {

            if (intakeReversing) {

                intake.startReverseContinuous();

            } else if (intakeRunning) {

                intake.startContinuous();

            } else if (intake.isContinuous() || intake.isBusy()) {

                intake.stopContinuous();
            }
        }
    }

    private void updateShooter(ShooterMode requestedMode) {

        if (shooterTwoMotors == null) {

            shooterMode = ShooterMode.OFF;
            shooterSequenceActive = false;
            activeTargetRPM = 0.0;

            return;
        }
        /*
         * Reinicia o relógio sempre que o modo muda.
         */
        if (requestedMode != shooterMode) {

            shooterMode = requestedMode;

            if (shooterMode == ShooterMode.OFF) {

                shooterSequenceActive = false;

            } else {

                shooterSequenceActive = true;
                shooterStartTimeMs = System.currentTimeMillis();
            }
        }

        if (shooterMode == ShooterMode.MANUAL) {
            activeTargetRPM = ShooterConfig.MANUAL_SHOOTER_RPM;

            shooterTwoMotors.setRPM(
                    ShooterConfig.MANUAL_LEFT_SHOOTER_RPM,
                    ShooterConfig.MANUAL_RIGHT_SHOOTER_RPM
            );
            shooterTwoMotors.setRawPower(1.0);

        } else if(shooterMode == ShooterMode.REVERSE){
            activeTargetRPM = ShooterConfig.REVERSE_SHOOTER_RPM;

            shooterTwoMotors.setReverseRPM(ShooterConfig.REVERSE_SHOOTER_RPM);

        }else if (shooterMode == ShooterMode.AUTO) {

            if (hasFreshCameraShooterSolution()) {
                activeTargetRPM = lastTargetRPM;
            } else {
                activeTargetRPM = ShooterConfig.MANUAL_SHOOTER_RPM;
            }

            shooterTwoMotors.setRPM(activeTargetRPM);

        } else {

            activeTargetRPM = 0.0;
            shooterTwoMotors.stop();
        }
    }

    private void updateBallRetainer() {

        if (ballRetainer != null && ballRetainer.isConfigured()) {

            if (gamepad2.a) {

                ballRetainer.block();

            } else {

                ballRetainer.release();
            }
        }
    }

    private void sendTelemetry() {

        telemetry.addLine("----- CONTROLES -----");

        telemetry.addData("LB shooter manual", gamepad2.left_bumper);
        telemetry.addData("LT autoshooter", gamepad2.left_trigger);
        telemetry.addData("RB intake frente", gamepad2.right_bumper);
        telemetry.addData("RT intake reverso", gamepad2.right_trigger);
        telemetry.addData("X alinhamento/parada", alignmentRequested);

        telemetry.addLine("----- MIRA -----");

        telemetry.addData("AprilTag visível", vision != null && vision.hasTarget());
        telemetry.addData("Shooter no RPM alvo", shooterTwoMotors != null && shooterTwoMotors.isAtTargetRPM());
        telemetry.addData("Alinhado", alignedWithTag);
        telemetry.addData("Heading", drive.getHeadingDegrees());

        if (vision != null && vision.hasTarget()) {

            telemetry.addData("Tag ID", vision.getTargetId());
            telemetry.addData("Bearing", vision.getBearingDegrees());
            telemetry.addData("Range (in)", vision.getRangeInches());
            telemetry.addData("Range cru (cm)", vision.getRangeInches() * 2.54);
            telemetry.addData("Y frontal (cm)", vision.getForwardInches() * 2.54);
            telemetry.addData("X lateral (cm)", vision.getSideInches() * 2.54);
        }

        telemetry.addLine("----- SHOOTER -----");

        telemetry.addData("Modo", shooterMode);
        telemetry.addData("Solução atual da câmera", hasFreshCameraShooterSolution());
        telemetry.addData("Auto aguardando câmera", autoShooterWaitingForCamera);
        telemetry.addData("Distância câmera crua (cm)", lastRawCameraDistanceCm);
        telemetry.addData("Distância corrigida sem filtro (cm)", lastUnfilteredShooterDistanceCm);
        telemetry.addData("Distância usada pelo auto (cm)", lastShooterDistanceCm);
        telemetry.addData("Distância dentro da tabela", shooterTable.isDistanceInsideTable(lastShooterDistanceCm));
        telemetry.addData("RPM calculado pela câmera", lastTargetRPM);
        telemetry.addData("RPM aplicado", activeTargetRPM);

        telemetry.addLine("----- SHOOTER MOTORES -----");

        if (shooterTwoMotors != null) {

            telemetry.addData("RPM atual médio", shooterTwoMotors.getCurrentRPM());
            telemetry.addData("RPM motor esquerdo", shooterTwoMotors.getLeftRPM());
            telemetry.addData("RPM motor direito", shooterTwoMotors.getRightRPM());
            telemetry.addData("RPM eixo esquerdo", shooterTwoMotors.getLeftMotorRPM());
            telemetry.addData("RPM eixo direito", shooterTwoMotors.getRightMotorRPM());
            telemetry.addData("Ticks/s esquerdo", shooterTwoMotors.getLeftTicksPerSecond());
            telemetry.addData("Ticks/s direito", shooterTwoMotors.getRightTicksPerSecond());
            telemetry.addData("Encoder esquerdo", shooterTwoMotors.getLeftEncoderPosition());
            telemetry.addData("Encoder direito", shooterTwoMotors.getRightEncoderPosition());
            telemetry.addData("RPM alvo esquerdo", shooterTwoMotors.getLeftTargetRPM());
            telemetry.addData("RPM alvo direito", shooterTwoMotors.getRightTargetRPM());
            telemetry.addData("Shooter pronto", shooterTwoMotors.isAtTargetRPM());
            telemetry.addData("Y shooter reverso", gamepad2.y);
            telemetry.addData(
                    "Shooter no RPM alvo", shooterTwoMotors != null
                            && shooterMode != ShooterMode.REVERSE
                            && shooterTwoMotors.isAtTargetRPM()
            );

        } else {

            telemetry.addLine("ShooterTwoMotors não disponível");
        }

        telemetry.addLine("----- INTAKE -----");
        telemetry.addData("Intake ligado", intakeRunning);
        telemetry.addData("Intake reverso", intakeReversing);
        telemetry.addData("Intake bloqueado pelo Shooter", intakeBlockedByShooter);
        telemetry.addData("Atraso AUTO ativo", autoIntakeDelayActive);
        telemetry.addData("Tempo restante Intake", "%d ms", autoIntakeDelayRemainingMs);

        telemetry.addLine("----- BALLRETAINER -----");
        telemetry.addData("Ball Retainer", ballRetainer != null ? ballRetainer.getState() : "DESATIVADO");

        if (hasFreshCameraShooterSolution() && !shooterTable.isDistanceInsideTable(lastShooterDistanceCm)) {

            telemetry.addLine("ATENÇÃO: distância fora da tabela; " + "usando o limite mais próximo.");
        }

        if (shooterTwoMotors != null) {
            long elapsedMs = shooterSequenceActive ? System.currentTimeMillis() - shooterStartTimeMs : 0;
            telemetry.addData("Tempo do Shooter (ms)", elapsedMs);
        }

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