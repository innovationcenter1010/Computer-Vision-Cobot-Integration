from cobot_application.cobot import motoman_cpp
from cobot_application.vision_system.yolo_obb_detector import OBBDetection

import asyncio
import time
from enum import Enum,auto
from dataclasses import dataclass
from typing import List

class Cobot_Job(Enum):
    move = auto()
    engage_gripper = auto()
    disengage_gripper = auto()
    add_trajectory_point = auto()
    reached_target_check = auto()


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
                    status_ok = result.status.is_ok
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

        tooling_offset_z = self.cobot_configs["tool_head_length_mm"]
        tooling_offset_xy = self.cobot_configs["tool_head_xy_offset"]
        compression_offset = self.cobot_configs["tool_head_compression_distance_mm"]

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, -100, 180, 0, 0)
        cobot_address_can_position = motoman_cpp.Position(can.real_x, can.real_y,100, 180, 0, 0)
        cobot_up = motoman_cpp.Position(can.real_x, can.real_y,500, 180, 0, 0)
        pickup_trajectory = [cobot_address_can_position,cobot_pickup_position,cobot_up]

        for position in pickup_trajectory:
            movement_report = await self.goto(position)
            pick_and_place_report.add_report(movement_report)

        #self._set_gripper(GRIPPER_ON)

        movement_report = await self.goto(self.target)
        pick_and_place_report.add_report(movement_report)


        #self._set_gripper(GRIPPER_OFF)

        await self._assert_sensor(pick_and_place_report,self.sensors["orientation"],True)

        return pick_and_place_report
    
    async def goto(self,position):
        last_result = self.robot.add_point_to_trajectory(position)

        goto_report = Cobot_Operation_Report()
        goto_report.add(Cobot_Job.add_trajectory_point,last_result)


        if (last_result.status.status_code == 0):
            self.robot.motion_start()
            while (last_result.status.is_ok and not last_result.payload):
                last_result = self.robot.has_reached_target()
                await asyncio.sleep(0.2)

        goto_report.add(Cobot_Job.reached_target_check,last_result)

        return goto_report

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
        for read_attempt in range (5):
            actual = self.robot.read_general_output_bit(sensor.group, sensor.bit).payload
            if(actual == expected):
                print("sensor assert passed!",flush=True)
                break

            await asyncio.sleep(1)
            print("sensor assert failed")
        
        report.add_sensor_assert(sensor,expected,actual)

        return actual == expected
    



    



