package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.GyroSensor

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

internal class MecanumDrive(hardwareMap: HardwareMap) {
    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "front_left")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "front_right")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val lift: DcMotor = hardwareMap.get(DcMotor::class.java, "lift")
    // private val blinkinLedDriver: RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        rearRight.direction = DcMotorSimple.Direction.REVERSE

        // blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE)
    }

    fun drive(x: Double, y: Double, z: Double) {
        frontLeft.power = y + z + x
        frontRight.power = y - z - x
        rearLeft.power = y + z - x
        rearRight.power = y - z + x
    }

    fun moveLift(power: Double) {
        lift.power = power
    }
}

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
internal class MecanumTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            val x = (-gamepad1.left_stick_x).toDouble()
            val y = gamepad1.left_stick_y.toDouble()
            val z = (-gamepad1.right_stick_x).toDouble()
            driver.drive(x, y, z)
            driver.moveLift(gamepad2.left_stick_y.toDouble())
        }
    }
}

@Autonomous(name = "Mecanum Autonomous", group = "Autonomous")
internal class MecanumAutonomus : LinearOpMode() {
    companion object {
        private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private const val LABEL_GOLD_MINERAL = "Gold Mineral"
        private const val LABEL_SILVER_MINERAL = "Silver Mineral"
        private const val VUFORIA_KEY = "AWalSLT/////AAABme4BfTt4y0Wnrn7kzOornkwjbHAMMZnWYYe0WOYGLbmQ914wZ8gozbWoSgsLHUQ+asiEx9VUOuDMkr4LsB5hB3iyn/JZM7NB1fG15dZSIXVDXevQ2iy+6zaVS8q0JZ2dpvugMWHEpsrmadtlA13znd7nTwhvVoc+2gGc2hNb5k6G4qe4l3jusYi3KGXTkzLNvjjYuQGP8xsBkGkXY3oHt6LNwdBy9EAcxRYmKEQUoGOOSNAL5vqB2ZHN2pKrlxXJs0BMbyhm1U4PTPYriGCb2rBmgiBvcJmfz9T/vVLCpL3T9mV1GByU38J1aMzj6QrQKWD6/ImZYMchpb2yG5Xq0vb1WyFGR9IKp6eaAJ0XQL+q"
    }

    private var tfod: TFObjectDetector? = null
    private val driver = MecanumDrive(hardwareMap)
    private val gyro = hardwareMap.get(GyroSensor::class.java, "gyro")

    override fun runOpMode() {
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
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            driver.drive(0.0, 0.0, 0.0)

            telemetry.addData("Gold Mineral Position", runRecognition())
            telemetry.update()
        }

        tfod?.shutdown()
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