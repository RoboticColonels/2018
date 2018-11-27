package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

internal class MecanumDrive(hardwareMap: HardwareMap) {
    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "front_left")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "front_right")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_right")
    private val lift: DcMotor = hardwareMap.get(DcMotor::class.java, "lift")
    private val latch: Servo = hardwareMap.get(Servo::class.java, "latche_servo")
    private val leftCol: DcMotor = hardwareMap.get(DcMotor::class.java, "collector_left")
    private val rightCol: DcMotor = hardwareMap.get(DcMotor::class.java, "collector_right")
    private val collector: DcMotor = hardwareMap.get(DcMotor::class.java, "collector")

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        rearRight.direction = DcMotorSimple.Direction.REVERSE

        rightCol.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(x: Double, y: Double, r: Double) {
        frontLeft.power = x + y + r
        frontRight.power = x - y - r
        rearLeft.power = x - y + r
        rearRight.power = x + y - r
    }

    fun setLift(power: Double) {
        lift.power = power
    }

    fun setLatch(position: Double) {
        latch.position = position
    }

    fun moveCollector(power: Double) {
        collector.power = power
    }

    fun setCollectorSpin(power: Double) {
        leftCol.power = power
        rightCol.power = power
    }
}

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
internal class MecanumTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            val x = -gamepad1.left_stick_y.toDouble()
            val y = gamepad1.left_stick_x.toDouble()
            val r = -gamepad1.right_stick_x.toDouble()
            driver.drive(x, y, r)

            driver.setLift((gamepad2.right_trigger - gamepad2.left_trigger).toDouble())

            if (gamepad2.right_bumper) {
                driver.setLatch(1.0)
            } else if (gamepad2.left_bumper) {
                driver.setLatch(-1.0)
            } else {
                driver.setLatch(0.0)
            }

            driver.moveCollector(-gamepad2.left_stick_y.toDouble())
            if (gamepad2.a) {
                driver.setCollectorSpin(1.0)
            } else if (gamepad2.b) {
                driver.setCollectorSpin(-1.0)
            } else {
                driver.setCollectorSpin(0.0)
            }

            idle()
        }
    }
}

@Autonomous(name = "Auto Hang", group = "Autonomous")
internal class MecanumAutonomusHang : LinearOpMode() {
    companion object {
        private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private const val LABEL_GOLD_MINERAL = "Gold Mineral"
        private const val LABEL_SILVER_MINERAL = "Silver Mineral"
        private const val VUFORIA_KEY = "AWalSLT/////AAABme4BfTt4y0Wnrn7kzOornkwjbHAMMZnWYYe0WOYGLbmQ914wZ8gozbWoSgsLHUQ+asiEx9VUOuDMkr4LsB5hB3iyn/JZM7NB1fG15dZSIXVDXevQ2iy+6zaVS8q0JZ2dpvugMWHEpsrmadtlA13znd7nTwhvVoc+2gGc2hNb5k6G4qe4l3jusYi3KGXTkzLNvjjYuQGP8xsBkGkXY3oHt6LNwdBy9EAcxRYmKEQUoGOOSNAL5vqB2ZHN2pKrlxXJs0BMbyhm1U4PTPYriGCb2rBmgiBvcJmfz9T/vVLCpL3T9mV1GByU38J1aMzj6QrQKWD6/ImZYMchpb2yG5Xq0vb1WyFGR9IKp6eaAJ0XQL+q"
    }

    private var tfod: TFObjectDetector? = null

    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        run {
            val parameters = VuforiaLocalizer.Parameters()
            parameters.vuforiaLicenseKey = VUFORIA_KEY
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
            val vuforia = ClassFactory.getInstance().createVuforia(parameters)
            val tfodMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)
            val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
            tfod!!.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL)
        }

        runRecognition()

        waitForStart()

        driver.drive(0.2, 0.0, 0.0)

        driver.setLift(1.0)

        sleep(1500)

        driver.drive(0.0, 0.0, 0.0)

        driver.setLift(0.0)

        driver.setLatch(0.0)

        sleep(4000)

        driver.drive(-0.5, 0.0, 0.0)

        sleep(2000)

        driver.drive(0.0, 0.0, 0.0)

        driver.setLift(-1.0)

        driver.drive(0.5, 0.0, 0.0)

        // driver.setDump(1.0)

        sleep(1000)

        driver.drive(0.0, 0.0, 0.0)

        driver.setLift(0.0)

        driver.setLatch(0.0)

        tfod!!.shutdown()
    }

    private fun runRecognition(): String? {
        val updatedRecognitions = tfod?.updatedRecognitions
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size)
            if (updatedRecognitions.size == 3) {
                var goldMineralX = -1
                var silverMineral1X = -1
                var silverMineral2X = -1
                for (recognition in updatedRecognitions) {
                    if (recognition.label == LABEL_GOLD_MINERAL) {
                        goldMineralX = recognition.left.toInt()
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = recognition.left.toInt()
                    } else {
                        silverMineral2X = recognition.left.toInt()
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        return "left"
                    }
                    if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        return "right"
                    }
                    return "center"
                }
            }
        }
        return null
    }
}

@Autonomous(name = "Auto Crater", group = "Autonomous")
internal class MecanumAutonomusCrater : LinearOpMode() {
    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        waitForStart()

        driver.drive(-0.4, 0.0, 0.0)

        sleep(4000)

        driver.drive(0.0, 0.0, 0.0)
    }
}