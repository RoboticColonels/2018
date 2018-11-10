//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//
//import java.util.List;
//
//class MecanumDrive {
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor rearLeft;
//    private DcMotor rearRight;
//    private DcMotor lift;
//    private RevBlinkinLedDriver blinkinLedDriver;
//
//
//    MecanumDrive(HardwareMap hardwareMap) {
//        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
//        frontRight = hardwareMap.get(DcMotor.class, "front_right");
//        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
//        rearRight = hardwareMap.get(DcMotor.class, "rear_right");
//
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        rearRight.setDirection(DcMotor.Direction.REVERSE);
//
//        lift = hardwareMap.get(DcMotor.class, "lift");
//
//        // blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        // blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//    }
//
//    void drive(double x, double y, double z) {
//        frontLeft.setPower(y + z + x);
//        frontRight.setPower(y - z - x);
//        rearLeft.setPower(y + z - x);
//        rearRight.setPower(y - z + x);
//    }
//
//    void moveLift(double power) {
//        lift.setPower(power);
//    }
//}
//
//@TeleOp(name="Mecanum TeleOp", group="TeleOp")
//class MecanumTeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        MecanumDrive driver = new MecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double x =-gamepad1.left_stick_x;
//            double y = gamepad1.left_stick_y;
//            double z = -gamepad1.right_stick_x;
//            driver.drive(x, y, z);
//            driver.moveLift(gamepad2.left_stick_y);
//        }
//    }
//}
//
//@Autonomous(name="Mecanum Autonomous", group="Autonomous")
//class MecanumAutonomus extends LinearOpMode {
//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
//    private static final String VUFORIA_KEY = "AWalSLT/////AAABme4BfTt4y0Wnrn7kzOornkwjbHAMMZnWYYe0WOYGLbmQ914wZ8gozbWoSgsLHUQ+asiEx9VUOuDMkr4LsB5hB3iyn/JZM7NB1fG15dZSIXVDXevQ2iy+6zaVS8q0JZ2dpvugMWHEpsrmadtlA13znd7nTwhvVoc+2gGc2hNb5k6G4qe4l3jusYi3KGXTkzLNvjjYuQGP8xsBkGkXY3oHt6LNwdBy9EAcxRYmKEQUoGOOSNAL5vqB2ZHN2pKrlxXJs0BMbyhm1U4PTPYriGCb2rBmgiBvcJmfz9T/vVLCpL3T9mV1GByU38J1aMzj6QrQKWD6/ImZYMchpb2yG5Xq0vb1WyFGR9IKp6eaAJ0XQL+q";
//
//    @Override
//    public void runOpMode() {
//        // MecanumDrive driver = new MecanumDrive(hardwareMap);
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // driver.drive(0, 0, 0);
//
//            if (tfod != null) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() == 3) {
//                        int goldMineralX = -1;
//                        int silverMineral1X = -1;
//                        int silverMineral2X = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                goldMineralX = (int) recognition.getLeft();
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) recognition.getLeft();
//                            } else {
//                                silverMineral2X = (int) recognition.getLeft();
//                            }
//                        }
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Left");
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Right");
//                            } else {
//                                telemetry.addData("Gold Mineral Position", "Center");
//                            }
//                        }
//                    }
//                    telemetry.update();
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//    }
//}