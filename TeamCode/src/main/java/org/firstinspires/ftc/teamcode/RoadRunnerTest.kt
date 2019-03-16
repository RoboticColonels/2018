package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.hardware.motors.NeveRest20Gearmotor
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

class RoadRunnerTestDrive(hardwareMap: HardwareMap) : MecanumDrive(1.0) {
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor::class.java)
    private val TICKS_PER_REV = MOTOR_CONFIG.ticksPerRev
    private val WHEEL_RADIUS = 4
    private val GEAR_RATIO = 0.72

    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "front_left")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "front_right")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_right")
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")

    private val motors = listOf(frontLeft, rearLeft, rearRight, frontRight)

    private val constraints: DriveConstraints? = null
    private val follower: TrajectoryFollower? = null

    override fun getExternalHeading(): Double {
        return imu.angularOrientation.firstAngle.toDouble()
    }

    override fun getWheelPositions(): List<Double> {
        return motors.map {
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
    override fun runOpMode() {
        val drive = RoadRunnerTestDrive(hardwareMap)

        val trajectory = TrajectoryLoader.load("test_trajectory")

        waitForStart()

        drive.followTrajectory(trajectory)
        while (!isStopRequested && drive.isFollowingTrajectory()) {
            drive.update()
        }
    }
}