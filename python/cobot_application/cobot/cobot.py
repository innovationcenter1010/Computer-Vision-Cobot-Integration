from cobot_application.cobot import motoman_cpp
from cobot_application.vision_system.yolo_obb_detector import OBBDetection

import asyncio
import time
from enum import Enum,auto
from dataclasses import dataclass
from typing import List
import copy
import math



SENSOR_READ_FREQUENCY = 0.2 #seconds
SENSOR_READ_ATTEMPTS = 5 
INTERPOLATION_INVALID_ALARM_CODE = 4684;


class Cobot_Job(Enum):
    move = auto()
    engage_gripper = auto()
    disengage_gripper = auto()
    add_trajectory_point = auto()
    reached_target_check = auto()
    motion_start = auto()


class Cobot_Operation_Status(Enum):
    job_success = 0
    job_failure = 1

@dataclass 
class Cobot_Sensor:
    name: str
    group: int
    bit: int
    
@dataclass
class Sensor_Assert:
    sensor: Cobot_Sensor
    expected: bool
    actual: bool
    passed: bool

@dataclass
class PositionOffset:
    x: float = 0.0
    y: float = 0.0
    original_z_rotation_deg: float = 0.0

    def is_empty(self):
        return self.x == 0.0 and self.y == 0.0

    def inverted(self, current_z_rotation_deg: float):

        delta_deg = (
            current_z_rotation_deg
            - self.original_z_rotation_deg
        )

        theta = math.radians(delta_deg)

        inv_x = -self.x
        inv_y = -self.y

        return PositionOffset(
            x=inv_x * math.cos(theta) - inv_y * math.sin(theta),
            y=inv_x * math.sin(theta) + inv_y * math.cos(theta),
            original_z_rotation_deg=current_z_rotation_deg,
        )
    


class Cobot_Operation_Report:

    def __init__(self):
        self.results: List[motoman_cpp.robot_result] = []
        self.jobs: List[Cobot_Job] = []
        self.sensor_asserts: List[Sensor_Assert] = []

        self.overall_status: Cobot_Operation_Status = (
            Cobot_Operation_Status.job_success
        )

    def add(self, job: Cobot_Job, result: motoman_cpp.robot_result) -> None:

        self.jobs.append(job)
        self.results.append(result)

        if not result.status.is_ok:
            self.overall_status = Cobot_Operation_Status.job_failure
    

    def add_report(self,report: "Cobot_Operation_Report",) -> None:

        self.jobs.extend(report.jobs)
        self.results.extend(report.results)
        self.sensor_asserts.extend(report.sensor_asserts)

        if report.overall_status == Cobot_Operation_Status.job_failure:
            self.overall_status = Cobot_Operation_Status.job_failure

    def add_sensor_assert(
        self,
        sensor: Cobot_Sensor,
        expected: bool,
        actual: bool,
    ) -> None:

        passed = (expected == actual)

        new_assert = Sensor_Assert(sensor=sensor,expected=expected,actual=actual,passed=passed)
        self.sensor_asserts.append(new_assert)
        
        if not passed:
            self.overall_status = Cobot_Operation_Status.job_failure

    def __str__(self) -> str:
        lines = []

        lines.append("=" * 60)
        lines.append("COBOT OPERATION REPORT")
        lines.append("=" * 60)
        lines.append(f"Overall Status : {self.overall_status.name}")
        lines.append(f"Jobs Executed  : {len(self.jobs)}")
        lines.append(f"Sensor Asserts : {len(self.sensor_asserts)}")
        lines.append("")

        # Job Results
        if self.jobs:
            lines.append("JOB RESULTS")
            lines.append("-" * 60)

            for idx, (job, result) in enumerate(
                zip(self.jobs, self.results),
                start=1,
            ):
                try:
                    status_ok = result.status.is_ok()
                except Exception:
                    status_ok = "UNKNOWN"

                try:
                    status_code = result.status.status_code
                except Exception:
                    status_code = "N/A"

                try:
                    payload = result.payload
                except Exception:
                    payload = "N/A"

                lines.append(
                    f"[{idx:02d}] {job.name:<25} "
                    f"ok={status_ok} "
                    f"code={status_code} "
                    f"payload={payload}"
                )

            lines.append("")

        # Sensor Assertions
        if self.sensor_asserts:
            lines.append("SENSOR ASSERTS")
            lines.append("-" * 60)

            for idx, assertion in enumerate(
                self.sensor_asserts,
                start=1,
            ):
                result_text = "PASS" if assertion.passed else "FAIL"

                lines.append(
                    f"[{idx:02d}] {assertion.sensor.name:<20} "
                    f"(G{assertion.sensor.group}:B{assertion.sensor.bit}) "
                    f"expected={assertion.expected} "
                    f"actual={assertion.actual} "
                    f"[{result_text}]"
                )

            lines.append("")

        return "\n".join(lines)


GRIPPER_ON = True
GRIPPER_OFF = False




class Cobot:

    #cobot will be initialized with the dictionary cobot configs
    #one of the sub dictionaies for the configurations json
    def __init__(self, cobot_configs):
        self.cobot_configs = cobot_configs
        self.robot = motoman_cpp.MotomanRobot(cobot_configs["cobot_ip"])

        self.sensors = {}
        self.sensors["orientation"] = Cobot_Sensor("orientation",1,3)
        
        

        for attempt in range(5):
            result = self.robot.connect()
            if result.status.is_ok:
                print("Connected to robot!!")
                break
            else:
                print(f"Connection attempt {attempt + 1} failed. Retrying...")
                time.sleep(0.1)


        self.gripper_io_group = 1
        self.gripper_io_bit = 2

        #cast the position data from configs.JSO into the position type compatible with the cobot 
        self.target = motoman_cpp.Position(*(self.cobot_configs["cobot_place_positions"])["place_position_a"])

    

    async def pick_and_place(self, can: OBBDetection):

        pick_and_place_report = Cobot_Operation_Report()
        interpolation_offset = PositionOffset()

        tooling_offset_z = self.cobot_configs["tool_head_length_mm"]
        tooling_offset_xy = self.cobot_configs["tool_head_xy_offset"]
        compression_offset = self.cobot_configs["tool_head_compression_distance_mm"]

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, -100, 180, 0, 0)
        cobot_address_can_position = motoman_cpp.Position(can.real_x, can.real_y,100, 180, 0, 0)
        cobot_up = motoman_cpp.Position(can.real_x, can.real_y,500, 180, 0, 0)

        pickup_trajectory = [
            [cobot_address_can_position,None],
            [cobot_pickup_position,interpolation_offset],
            [cobot_up,None]]

        for position in pickup_trajectory:
            movement_report = await self.goto(*position)
            pick_and_place_report.add_report(movement_report)

        #self._set_gripper(GRIPPER_ON)

        movement_report = await self.goto(self.target,interpolation_offset)
        pick_and_place_report.add_report(movement_report)


        #self._set_gripper(GRIPPER_OFF)

        await self._assert_sensor(pick_and_place_report,self.sensors["orientation"],True)

        return pick_and_place_report
    
    async def goto(self,position,offset: PositionOffset | None = None):

        goto_report = Cobot_Operation_Report()

        target_position = copy.deepcopy(position)

        #
        # If offset contains data,
        # apply its inverse to the target.
        #
        if offset is not None and not offset.is_empty():

            inverted_offset = offset.inverted(
                target_position.rz
            )

            target_position.x += inverted_offset.x
            target_position.y += inverted_offset.y

        last_result = self.robot.add_point_to_trajectory(target_position)

        goto_report.add(Cobot_Job.add_trajectory_point,last_result)

        if last_result.status.status_code == 0:

            last_result = self.robot.motion_start()

            goto_report.add(
                Cobot_Job.motion_start,
                last_result
            )

            if last_result.status.status_code == 0:

                last_result = self.robot.has_reached_target()

                while (
                    not last_result.payload
                    and last_result.status.status_code == 0
                ):
                    await asyncio.sleep(0.2)

                    last_result = (
                        self.robot.has_reached_target()
                    )

        goto_report.add(Cobot_Job.reached_target_check,last_result)

        # Interpolation failure.
        if (last_result.status.status_code == INTERPOLATION_INVALID_ALARM_CODE):

            generated_offset = (self._generate_interpolation_offset(target_position))

            # Store generated offset if caller
            # provided an empty offset object.
            if (offset is not None and offset.is_empty()):
                offset.x = generated_offset.x
                offset.y = generated_offset.y
                offset.original_z_rotation_deg = (
                    generated_offset.original_z_rotation_deg
                )

            retry_position = copy.deepcopy(
                target_position
            )

            retry_position.x += generated_offset.x
            retry_position.y += generated_offset.y

            retry_report = await self.goto(retry_position,None)

            goto_report.add_report(retry_report)

            return goto_report
    
    async def goto_nearby_position(self, position, offset: PositionOffset):
        """
        Creates a nearby target position using the supplied offset
        and executes a goto on that new position.
        """

        nearby_position = copy.deepcopy(position)

        nearby_position.x += offset.x
        nearby_position.y += offset.y

        return await self.goto(nearby_position, offset=None)
    
    def _generate_interpolation_offset(
        self,
        position,
        step_mm=50
        ):

        return PositionOffset(
            x=-math.copysign(
                min(step_mm, abs(position.x)),
                position.x,
            ),
            y=-math.copysign(
                min(step_mm, abs(position.y)),
                position.y,
            ),
            original_z_rotation_deg=position.rz,
            )

    def disconnect(self):
        self.robot.disconnect()

    def _set_gripper(self, state: bool):
        gripper_report = Cobot_Operation_Report()


        gripper_job = Cobot_Job.disengage_gripper
        if state: gripper_job = Cobot_Job.engage_gripper

        result = self.robot.write_general_output_bit(self.gripper_io_group, self.gripper_io_bit, state)

        gripper_report.add(gripper_job,result)

    async def _assert_sensor(
        self,
        report: Cobot_Operation_Report,
        sensor: Cobot_Sensor,
        expected: bool,
        ) -> bool:

        actual = False
        
        for read_attempt in range (SENSOR_READ_ATTEMPTS):

            last_result = self.robot.read_general_output_bit(sensor.group, sensor.bit).payload

            actual = True
            
            if(actual == expected):break

            await asyncio.sleep(SENSOR_READ_FREQUENCY)
        
        report.add_sensor_assert(sensor,expected,actual)

        return actual == expected
    



    



