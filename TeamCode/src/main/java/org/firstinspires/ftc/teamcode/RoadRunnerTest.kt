package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig
import java.io.IOException
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.module.kotlin.KotlinModule
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector


object TrajectoryLoader {
    private val MAPPER = ObjectMapper(YAMLFactory())

    init {
        MAPPER.registerModule(KotlinModule(512))
    }

    @Throws(IOException::class)
    fun loadConfig(name: String): TrajectoryConfig {
        val inputStream = AppUtil.getDefContext().assets.open("trajectory/$name.yaml")
        return MAPPER.readValue(inputStream, TrajectoryConfig::class.java)
    }

    @Throws(IOException::class)
    fun load(name: String): Trajectory {
        return loadConfig(name).toTrajectory()
    }
}

class RoadRunnerTestDrive(hardwareMap: HardwareMap) : MecanumDrive(13.0) {
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor::class.java)
    private val TICKS_PER_REV = MOTOR_CONFIG.ticksPerRev
    companion object {
        private const val WHEEL_RADIUS = 4
        private const val GEAR_RATIO = 0.72
    }

    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "front_left")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "front_right")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_right")
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")

    private val constraints: DriveConstraints? = null
    private val follower: TrajectoryFollower? = null

    override fun getExternalHeading(): Double {
        // firstAngle = heading
        // secondAngle = roll
        // thirdAngle = pitch
        // our board is mounted sideways, so we use thirdAngle
        return imu.angularOrientation.thirdAngle.toDouble()
    }

    override fun getWheelPositions(): List<Double> {
        // this list has a specific order, don't heck it up pls
        // frontLeft, rearLeft, rearRight, frontRight
        return listOf(frontLeft, rearLeft, rearRight, frontRight).map {
            encoderTicksToInches(it.currentPosition)
        }
    }

    override fun setMotorPowers(fl: Double, rl: Double, rr: Double, fr: Double) {
        frontLeft.power = fl
        frontRight.power = fr
        rearLeft.power = rl
        rearRight.power = rr
    }

    private fun encoderTicksToInches(ticks: Int): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints!!)
    }

    fun followTrajectory(trajectory: Trajectory) {
        follower!!.followTrajectory(trajectory)
    }

    fun isFollowingTrajectory(): Boolean {
        return follower!!.isFollowing()
    }

    fun update() {
        updatePoseEstimate()
        follower!!.update(poseEstimate)
    }
}

@Autonomous(name = "RoadRunnerTest", group = "Autonomous")
internal class RoadRunnerTest : LinearOpMode() {
    companion object {
        private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private const val LABEL_GOLD_MINERAL = "Gold Mineral"
        private const val LABEL_SILVER_MINERAL = "Silver Mineral"
        private const val VUFORIA_KEY = "AWalSLT/////AAABme4BfTt4y0Wnrn7kzOornkwjbHAMMZnWYYe0WOYGLbmQ914wZ8gozbWoSgsLHUQ+asiEx9VUOuDMkr4LsB5hB3iyn/JZM7NB1fG15dZSIXVDXevQ2iy+6zaVS8q0JZ2dpvugMWHEpsrmadtlA13znd7nTwhvVoc+2gGc2hNb5k6G4qe4l3jusYi3KGXTkzLNvjjYuQGP8xsBkGkXY3oHt6LNwdBy9EAcxRYmKEQUoGOOSNAL5vqB2ZHN2pKrlxXJs0BMbyhm1U4PTPYriGCb2rBmgiBvcJmfz9T/vVLCpL3T9mV1GByU38J1aMzj6QrQKWD6/ImZYMchpb2yG5Xq0vb1WyFGR9IKp6eaAJ0XQL+q"
    }

    private val arm: DcMotor = hardwareMap.get(DcMotor::class.java, "arm")
    private val lift: DcMotor = hardwareMap.get(DcMotor::class.java, "lift")
    private val dump: DcMotor = hardwareMap.get(DcMotor::class.java, "dump")
    private val collector: DcMotor = hardwareMap.get(DcMotor::class.java, "collect")
    private val marker: Servo = hardwareMap.get(Servo::class.java, "marker")

    private var tfod: TFObjectDetector? = null

    init {
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun runOpMode() {
        val drive = RoadRunnerTestDrive(hardwareMap)

        val unlatch = drive.trajectoryBuilder()
                .splineTo(Pose2d(10.0, 10.0, 45.0))
                .splineTo(Pose2d(8.0, 12.0, 45.0))
                .build()

        val landerToLeft = drive.trajectoryBuilder()
                .splineTo(Pose2d(8.0, 12.0, 45.0))
                .splineTo(Pose2d(20.0, 42.0, 45.0))
                .build()

        val landerToCenter = drive.trajectoryBuilder()
                .splineTo(Pose2d(8.0, 12.0, 45.0))
                .splineTo(Pose2d(-30.0, 30.0, 45.0))
                .build()

        val landerToRight = drive.trajectoryBuilder()
                .splineTo(Pose2d(8.0, 12.0, 45.0))
                .splineTo(Pose2d(40.0, 18.0, 45.0))
                .build()

        val leftToCrater = drive.trajectoryBuilder()
                .splineTo(Pose2d(20.0, 42.0, 45.0))
                .splineTo(Pose2d(40.0, 42.0, 45.0))
                .build()

        val centerToCrater = drive.trajectoryBuilder()
                .splineTo(Pose2d())
                .splineTo(Pose2d(40.0, 42.0, 45.0))
                .build()

        val rightToCrater = drive.trajectoryBuilder()
                .splineTo(Pose2d())
                .splineTo(Pose2d(40.0, 42.0, 45.0))
                .build()

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

        waitForStart()

        val mineralPosition = runRecognition()
        tfod!!.deactivate()
        tfod!!.shutdown()

        // lower robot to ground
        lift.power = -1.0
        sleep(1100)
        lift.power = 0.0

        // unlatch
        drive.followTrajectory(unlatch)
        while (!isStopRequested && drive.isFollowingTrajectory()) {
            drive.update();
        }

        // drive to mineral
        when (mineralPosition) {
            "left" -> {
                drive.followTrajectory(landerToLeft)
            }
            "right" -> {
                drive.followTrajectory(landerToRight)
            }
            "center" -> {
                drive.followTrajectory(landerToCenter)
            }
        }
        while (!isStopRequested && drive.isFollowingTrajectory()) {
            drive.update()
        }

        // drive to crater
        when (mineralPosition) {
            "left" -> {
                drive.followTrajectory(leftToCrater)
            }
            "right" -> {
                drive.followTrajectory(rightToCrater)
            }
            "center" -> {
                drive.followTrajectory(centerToCrater)
            }
        }
        while (!isStopRequested && drive.isFollowingTrajectory()) {
            drive.update()
        }
    }

    private fun runRecognition(): String {
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

        // default position, can be whatever
        return "left"
    }
}