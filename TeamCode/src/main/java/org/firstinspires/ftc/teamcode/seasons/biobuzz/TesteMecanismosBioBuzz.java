package org.firstinspires.ftc.teamcode.seasons.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode de bancada para testar a rampa e o guarda-chuva isoladamente.
 *
 * É intencionalmente explícito e conservador: cada movimento exige uma nova
 * pressionada de botão, e nenhum mecanismo se move se estiver desabilitado ou
 * sem configuração física válida. Não inicializa Drivetrain, Shooter, Intake,
 * Localization ou câmera porque este teste deve isolar os dois atuadores.
 *
 * Controles (gamepad1):
 *   A = bloquear rampa; B = liberar rampa
 *   X = guarda-chuva para repouso; Y = guarda-chuva para frente
 *   LB = parar o movimento do guarda-chuva
 *
 * Antes de habilitar mecanismos, confirmar seus nomes e calibrar os valores
 * físicos nos arquivos *Config. Nenhum valor é inferido neste OpMode.
 */
@TeleOp(name = "Teste Mecanismos BIOBUZZ", group = "Test")
public class TesteMecanismosBioBuzz extends LinearOpMode {

    private BallRetainer ballRetainer;
    private Umbrella umbrella;

    @Override
    public void runOpMode() {
        if (BallRetainerConfig.ENABLED) {
            ballRetainer = new BallRetainer();
            ballRetainer.init(hardwareMap);
        }

        if (UmbrellaConfig.ENABLED) {
            umbrella = new Umbrella();
            umbrella.init(hardwareMap);
        }

        telemetry.addLine("Teste isolado dos mecanismos BIOBUZZ");
        telemetry.addData("Rampa habilitada", BallRetainerConfig.ENABLED);
        telemetry.addData("Rampa configurada", ballRetainer != null && ballRetainer.isConfigured());
        telemetry.addData("Guarda-chuva habilitado", UmbrellaConfig.ENABLED);
        telemetry.addData("Guarda-chuva configurado", umbrella != null && umbrella.isConfigured());
        telemetry.addLine("A: bloquear | B: liberar rampa");
        telemetry.addLine("X: repouso | Y: frente | LB: parar guarda-chuva");
        telemetry.addLine("Nenhum movimento ocorre com config ausente/invalida.");
        telemetry.update();

        waitForStart();

        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousLeftBumper = false;

        while (opModeIsActive()) {
            boolean aPressed = gamepad1.a && !previousA;
            boolean bPressed = gamepad1.b && !previousB;
            boolean xPressed = gamepad1.x && !previousX;
            boolean yPressed = gamepad1.y && !previousY;
            boolean leftBumperPressed = gamepad1.left_bumper && !previousLeftBumper;

            if (ballRetainer != null && ballRetainer.isConfigured()) {
                if (aPressed) {
                    ballRetainer.block();
                } else if (bPressed) {
                    ballRetainer.release();
                }
            }

            if (umbrella != null && umbrella.isConfigured()) {
                if (leftBumperPressed) {
                    umbrella.stop();
                } else if (xPressed) {
                    umbrella.moveToRest();
                } else if (yPressed) {
                    umbrella.moveToForward();
                }
            }

            telemetry.addData("Rampa habilitada", BallRetainerConfig.ENABLED);
            telemetry.addData("Rampa configurada", ballRetainer != null && ballRetainer.isConfigured());
            String retainerState = ballRetainer == null
                    ? "DESABILITADA"
                    : ballRetainer.isConfigured()
                        ? ballRetainer.getState().name()
                        : "SEM CONFIGURACAO VALIDA";
            telemetry.addData("Rampa estado", retainerState);
            telemetry.addData("Guarda-chuva habilitado", UmbrellaConfig.ENABLED);
            telemetry.addData("Guarda-chuva configurado", umbrella != null && umbrella.isConfigured());
            telemetry.addData("Guarda-chuva em movimento", umbrella != null && umbrella.isBusy());
            telemetry.addLine("A bloquear / B liberar / X repouso / Y frente / LB parar");
            telemetry.update();

            previousA = gamepad1.a;
            previousB = gamepad1.b;
            previousX = gamepad1.x;
            previousY = gamepad1.y;
            previousLeftBumper = gamepad1.left_bumper;
            idle();
        }

        if (umbrella != null) {
            umbrella.stop();
        }
    }
}
