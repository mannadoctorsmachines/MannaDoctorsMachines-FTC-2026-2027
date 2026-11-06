package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.safety.MotorSafetyManager;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.localization.GobildaOdometry;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.LocalizationConfig;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterTwoMotors;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.limelight.Limelight;

/**
 * Ponto central de composição do robô.
 *
 * O Robot é responsável por:
 * - criar os subsistemas universais;
 * - inicializar os subsistemas;
 * - atualizar os componentes que precisam de update();
 * - fornecer acesso aos subsistemas;
 * - executar a parada geral do robô.
 *
 * O Robot NÃO contém:
 * - lógica de estratégia;
 * - lógica específica da temporada;
 * - controle direto de motores;
 * - PID;
 * - cálculos de pose;
 * - processamento de AprilTag;
 * - comandos específicos de jogo.
 *
 * Arquitetura:
 *
 *     OpMode
 *        ↓
 *      Robot
 *        ↓
 *   Subsystems
 *        ↓
 *    Hardware
 */
public class Robot {

    private final Drivetrain drivetrain;

    /*
     * Apenas UMA implementação do Shooter é criada.
     *
     * Shooter:
     * implementação antiga / 1 motor
     *
     * ShooterTwoMotors:
     * implementação atual / 2 motores
     */
    //private final Shooter shooter;
    private final ShooterTwoMotors shooterTwoMotors;

    private final Intake intake;
    private final Localizer localization;

    private Vision vision;
    private final MotorSafetyManager motorSafety;

    private boolean initialized = false;

    public Robot() {
        motorSafety = new MotorSafetyManager();
        drivetrain = new Drivetrain();
        /*
         * O Robot escolhe qual implementação do Shooter
         * será utilizada.
         *
         * As duas classes continuam existindo no projeto,
         * mas nunca são inicializadas simultaneamente.
         */
        if (RConstants.USE_SHOOTER) {
            if (RConstants.USE_TWO_MOTOR_SHOOTER) {
                //shooter = null;
                shooterTwoMotors = new ShooterTwoMotors();

            } else {
                //shooter = new Shooter();
                shooterTwoMotors = null;
            }

        } else {

            //shooter = null;
            shooterTwoMotors = null;
        }

        intake = RConstants.USE_FEEDER ? new Intake() : null;
        localization = RConstants.USE_LOCALIZATION ? new GobildaOdometry() : null;
    }

    /**
     * Inicializa os componentes universais do robô.
     *
     * Ordem:
     *
     * Drivetrain
     * Localization
     * Shooter
     * Intake
     */
    public void init(HardwareMap hardwareMap) {

        drivetrain.init(hardwareMap);

        if (localization != null) {
            localization.init(hardwareMap);
        }

        /*
         * Apenas o Shooter selecionado pelo RConstants
         * será inicializado.
         */
//        if (shooter != null) {
//            shooter.init(hardwareMap);
//        }

        if (shooterTwoMotors != null) {
            shooterTwoMotors.init(hardwareMap);
        }

        if (intake != null) {
            intake.init(hardwareMap);
        }

        initialized = true;
    }

    /**
     * Inicializa o sistema de visão escolhido em RobotConfig.
     *
     * A Vision é inicializada separadamente porque nem todo
     * OpMode precisa utilizar câmera.
     */
    public void initVision(HardwareMap hardwareMap) {

        if (!RConstants.USE_CAMERA) {
            return;
        }

        switch (RobotConfig.VISION_PROVIDER) {

            case LIMELIGHT:
                vision = new Limelight();
                break;

            case APRILTAG_CAMERA:
            default:
                vision = new AprilTagCamera();
                break;
        }

        vision.init(hardwareMap);
    }

    /**
     * Atualiza os componentes que precisam de atualização periódica.
     *
     * O OpMode deve chamar este método uma vez por ciclo.
     */
    public void update() {

        if (!initialized) {
            return;
        }

        /*
         * Localização primeiro.
         */
        if (localization != null) {
            localization.update();
        }

        /*
         * Depois visão.
         *
         * Caso configurado, a visão pode fornecer uma correção
         * para o sistema de localização.
         */
        if (vision != null) {

            vision.update();

            if (localization != null && LocalizationConfig.ACCEPT_VISION_CORRECTIONS) {
                localization.correctPose(vision.getPoseObservation());
            }
        }

        /*
         * Intake possui atualização periódica.
         */
        if (intake != null) {
            intake.update();
        }
    }

    /**
     * Retorna o drivetrain universal.
     */
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * Retorna o Shooter antigo.
     *
     * Será null quando USE_TWO_MOTOR_SHOOTER = true.
     */
//    public Shooter getShooter() {
//        return shooter;
//    }

    /**
     * Retorna o ShooterTwoMotors.
     *
     * Será null quando USE_TWO_MOTOR_SHOOTER = false.
     */
    public ShooterTwoMotors getTwoShooter() {
        return shooterTwoMotors;
    }

    /**
     * Retorna o Intake.
     *
     * Será null quando USE_FEEDER = false.
     */
    public Intake getIntake() {
        return intake;
    }

    /**
     * Retorna o sistema de localização.
     *
     * Será null quando USE_LOCALIZATION = false.
     */
    public Localizer getLocalization() {
        return localization;
    }

    /**
     * Retorna a Vision.
     *
     * Será null quando:
     * - USE_CAMERA = false;
     * - initVision() ainda não foi chamado.
     */
    public Vision getVision() {
        return vision;
    }

    /**
     * Para todos os subsistemas ativos.
     */
    public void stopAll() {

        if (!initialized) {
            return;
        }

        drivetrain.stop();

        if (intake != null) {
            intake.stop();
        }

        if (shooterTwoMotors != null) {
            shooterTwoMotors.stop();
        }
    }

    /**
     * Fecha/libera a Vision.
     */
    public void closeVision() {

        if (vision != null) {
            vision.close();
        }
    }
}