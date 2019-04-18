package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

internal class MecanumDrive(hardwareMap: HardwareMap) {
    companion object {
        const val COUNTS_PER_ROTATION: Int = 657
    }
    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "leftFront")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rightFront")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "leftRear")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rightRear")
    private val arm: DcMotor = hardwareMap.get(DcMotor::class.java, "armRotate")
    private val lift: DcMotor = hardwareMap.get(DcMotor::class.java, "lift")
    private val dump: DcMotor = hardwareMap.get(DcMotor::class.java, "armExtend")
    private val collector: DcMotor = hardwareMap.get(DcMotor::class.java, "collectorIntake")
    // private val marker: Servo = hardwareMap.get(Servo::class.java, "marker")

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        rearRight.direction = DcMotorSimple.Direction.REVERSE

        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun drive(distance: Double, left: Double, right: Double) {
        frontLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rearLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rearRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        rearLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        frontRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        rearLeft.mode = DcMotor.RunMode.RUN_TO_POSITION

        frontLeft.power = left
        rearLeft.power = left
        frontRight.power = right
        rearRight.power = right

        val leftDist = (if (left < 0.0) -distance else distance) * COUNTS_PER_ROTATION
        val rightDist = (if (right < 0.0) -distance else distance) * COUNTS_PER_ROTATION

        frontLeft.targetPosition = leftDist.toInt()
        rearLeft.targetPosition = leftDist.toInt()
        frontRight.targetPosition = rightDist.toInt()
        rearRight.targetPosition = rightDist.toInt()
    }

    fun driveMec(x: Double, y: Double, r: Double) {
        frontLeft.power = x + y + r
        frontRight.power = x - y - r
        rearLeft.power = x - y + r
        rearRight.power = x + y - r
    }


    fun setArmPower(pow: Double) {
        arm.power = pow
    }

    fun setDumpPower(pow: Double) {
        dump.power = pow
    }

    fun setLiftPower(pow: Double) {
        lift.power = pow
    }

    fun setCollectPower(pow: Double) {
        collector.power = pow
    }

    fun setMarkerPosition(position: Double) {
        // marker.position = position
    }
}

@Autonomous(name = "AutoCraterNative", group = "Autonomous")
@Disabled
internal class MecanumAutonomusHang : LinearOpMode() {
    companion object {
        private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private const val LABEL_GOLD_MINERAL = "Gold Mineral"
        private const val LABEL_SILVER_MINERAL = "Silver Mineral"
        private const val VUFORIA_KEY = "AWalSLT/////AAABme4BfTt4y0Wnrn7kzOornkwjbHAMMZnWYYe0WOYGLbmQ914wZ8gozbWoSgsLHUQ+asiEx9VUOuDMkr4LsB5hB3iyn/JZM7NB1fG15dZSIXVDXevQ2iy+6zaVS8q0JZ2dpvugMWHEpsrmadtlA13znd7nTwhvVoc+2gGc2hNb5k6G4qe4l3jusYi3KGXTkzLNvjjYuQGP8xsBkGkXY3oHt6LNwdBy9EAcxRYmKEQUoGOOSNAL5vqB2ZHN2pKrlxXJs0BMbyhm1U4PTPYriGCb2rBmgiBvcJmfz9T/vVLCpL3T9mV1GByU38J1aMzj6QrQKWD6/ImZYMchpb2yG5Xq0vb1WyFGR9IKp6eaAJ0XQL+q"
    }

    private var lastDetection: String? = null
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
            tfodParameters.minimumConfidence = 0.30
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
            tfod!!.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL)
            tfod!!.activate()
        }

        val mineralPosition = runRecognition()
        tfod!!.deactivate()
        tfod!!.shutdown()

        waitForStart()

        // lower down
        driver.setLiftPower(-1.0)
        sleep(4300)
        driver.setLiftPower(0.0)

        // unlatch
        driver.driveMec(0.0, -0.5, 0.0)
        sleep(200)
        driver.driveMec(0.0, 0.0, 0.0)

        driver.setLiftPower(1.0)
        driver.setArmPower(1.0)

        // sample
        when (mineralPosition) {
            "left", null -> {
                driver.drive(0.3, 0.5, 0.5)
            }
            "right" -> {
                driver.drive(0.25, 0.5, 0.5)
            }
            "center" -> {
                driver.drive(0.65, 0.5, 0.5)
            }
        }

        driver.setLiftPower(0.0)
        driver.setArmPower(0.0)

        driver.drive(0.31, 0.5, 0.5)
        driver.drive(1.43, 0.5, -0.5)

//        val depotDrivePower = 0.7
//        when (mineralPosition) {
//            "left", null -> {
//                driver.drive(3.8, -depotDrivePower, -depotDrivePower)
//            }
//            "right" -> {
//                driver.drive(3.5, -depotDrivePower, -depotDrivePower)
//            }
//            "center" -> {
//                driver.drive(3.3, -depotDrivePower, -depotDrivePower)
//            }
//        }
//        sampleForwardAndBack(driver)

//        driver.drive(0.7, -0.5, 0.5)
//        driver.drive(1.8, 0.5, -0.5)
//
//        when (mineralPosition) {
//            "left", null -> {
//                driver.drive(3.8, -depotDrivePower, -depotDrivePower)
//            }
//            "right" -> {
//                driver.drive(3.0, -depotDrivePower, -depotDrivePower)
//            }
//            "center" -> {
//                driver.drive(3.2, -depotDrivePower, -depotDrivePower)
//            }
//        }
//
//        sleep(200)
//
//        driver.setMarkerPosition(-1.0)
//        sleep(2000)
//        driver.setMarkerPosition(0.5)
//
//        sleep(1000)
//
//        driver.drive(0.3, 0.5, -0.5)
//        driver.drive(4.75, 1.0, 1.0)
//
//        driver.setArmPower(0.5)
//        sleep(400)
//        driver.setArmPower(0.0)
    }

    private fun sampleForwardAndBack(driver: MecanumDrive) {
        driver.drive(0.69, 0.3, 0.3)
        driver.setCollectPower(1.0)
        sleep(1500)
        driver.setCollectPower(0.0)
        sleep(600)
    }

    private fun eagerRunRecognition(): String? {
        val updatedRecognitions = tfod?.updatedRecognitions
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size)
            var goldMineralX: Int? = null
            for (recognition in updatedRecognitions) {
                if (recognition.label == LABEL_GOLD_MINERAL) {
                    goldMineralX = recognition.right.toInt()
                }
            }
            if (goldMineralX == null) {
                return "right"
            }
            if (goldMineralX > 400) {
                return "center"
            }
            return "left"
        }
        return null
    }

    private fun runRecognition(): String? {
        val current = eagerRunRecognition()
        if (current != null) {
            lastDetection = current
        }
        telemetry.addData("detection", lastDetection)
        return lastDetection
    }
}