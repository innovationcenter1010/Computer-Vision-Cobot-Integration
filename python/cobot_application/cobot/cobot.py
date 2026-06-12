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
    sensor_read = auto()

class place_position(Enum):
    right = auto()
    left = auto()


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

    def __init__(self, name = "none"):
        self.results: List[motoman_cpp.robot_result] = []
        self.jobs: List[Cobot_Job] = []
        self.sensor_asserts: List[Sensor_Assert] = []
        self.overall_status: Cobot_Operation_Status = (Cobot_Operation_Status.job_success)
        self.job_title: str = name

    def add(self, job: Cobot_Job, result: motoman_cpp.robot_result) -> None:

        self.jobs.append(job)
        self.results.append(result)

        if not result.status.is_ok:
            self.overall_status = Cobot_Operation_Status.job_failure
    

    def add_report(self,report: "Cobot_Operation_Report",) -> None:

        if(report.jobs):self.jobs.extend(report.jobs)

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
        self.sensors["existance_check"] = Cobot_Sensor("existance_check",1,3)
        self.sensors["divot_check"] = Cobot_Sensor("divot_check",1,3)
        self.sensors["right_complete"] = Cobot_Sensor("right_complete",1,3)
        self.sensors["left_complete"] = Cobot_Sensor("left_complete",1,3)

        self.last_placed: place_position = place_position.left
    

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

        pick_and_place_report = Cobot_Operation_Report("PICK AND PLACE REPORT")
        interpolation_offset = PositionOffset()

        tooling_offset_z = self.cobot_configs["tool_head_length_mm"]
        tooling_offset_xy = self.cobot_configs["tool_head_xy_offset"]
        compression_offset = self.cobot_configs["tool_head_compression_distance_mm"]

        cobot_pickup_position = motoman_cpp.Position(can.real_x, can.real_y, 450, 180, 0, 0)
        cobot_address_can_position = motoman_cpp.Position(can.real_x, can.real_y,500, 180, 0, 0)
        cobot_up = motoman_cpp.Position(can.real_x, can.real_y,500, 180, 0, 0)

        pickup_trajectory = [
            [cobot_address_can_position,None],
            [cobot_pickup_position,interpolation_offset]]
        

        for position in pickup_trajectory:
            movement_report = await self.goto(*position)
            pick_and_place_report.add_report(movement_report)

        gripper_report = self._set_gripper(GRIPPER_ON)
        pick_and_place_report.add_report(gripper_report)

        movement_report = await self.goto(cobot_up,None)
        pick_and_place_report.add_report(movement_report)

        #move to the check position 
        movement_report = await self.goto(self.targets["check"],interpolation_offset)
        pick_and_place_report.add_report(movement_report)

        
        await self._assert_sensor(pick_and_place_report,self.sensors["existance_check"],True)
        divot_detection = await self._read_sensor(pick_and_place_report,self.sensors["divot_check"])

        right_detection = await self._read_sensor(pick_and_place_report,self.sensors["right_complete"])
        left_detection = await self._read_sensor(pick_and_place_report,self.sensors["left_complete"])

        place_trajectory = []
        just_placed: place_position

        if(self.last_placed == place_position.left and not right_detection):
            just_placed = place_position.right
            if(divot_detection):
                place_trajectory = self.target["right"]
            else:
                place_trajectory = self.target["right_flip"]

        elif(self.last_placed == place_position.right and not left_detection):
            just_placed = place_position.left
            if(not divot_detection):
                place_trajectory = self.target["left"]
            else:
                place_trajectory = self.target["left_flip"]


        for position in place_trajectory:
            movement_report = await self.goto(*position,interpolation_offset)
            pick_and_place_report.add_report(movement_report)

        self._set_gripper(GRIPPER_OFF)

        movement_report = await self.goto(self.target,interpolation_offset)
        pick_and_place_report.add_report(movement_report)


        #self._set_gripper(GRIPPER_OFF)

        await self._assert_sensor(pick_and_place_report,self.sensors["orientation"],True)

        return pick_and_place_report
    
    async def goto(self,position,offset: PositionOffset | None = None):

        goto_report = Cobot_Operation_Report("GOTO REPORT")

        target_position = position.copy()

        #
        # If offset contains data,
        # apply its inverse to the target.
        #
        if offset is not None and not offset.is_empty():

            inverted_offset = offset.inverted(target_position.rz)

            target_position.x += inverted_offset.x
            target_position.y += inverted_offset.y

        last_result = self.add_point_with_offset_recovery(goto_report,target_position,offset)

        goto_report.add(Cobot_Job.add_trajectory_point,last_result)

        if last_result.status.status_code == 0:

            last_result = self.robot.motion_start()

            goto_report.add(Cobot_Job.motion_start,last_result)

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
        
        return goto_report
    
    async def add_point_with_offset_recovery(
        self,
        report: Cobot_Operation_Report,
        target_position,
        interpolation_offset: PositionOffset,
        max_steps: int = 10,
        step_fraction: int = 100,
        ):

        base_x = target_position.x
        base_y = target_position.y

        # direction toward robot base (0,0)
        step_x = -base_x / step_fraction
        step_y = -base_y / step_fraction

        current_pos = target_position.copy()

        for i in range(max_steps + 1):

            result = self.robot.add_point_to_trajectory(current_pos)
            report.add(Cobot_Job.add_trajectory_point, result)

            status_code = result.status.status_code

            # SUCCESS → return immediately
            if status_code == 0:
                return report

            # If NOT interpolation error → stop immediately
            if status_code != 3:
                return report

            # LINEAR STEP toward base
            #current_pos = current_pos.copy()
            current_pos.x += step_x
            current_pos.y += step_y

            # Optional: apply external offset correction each iteration
            #if interpolation_offset is not None and not interpolation_offset.is_empty():
                #inverted = interpolation_offset.inverted(current_pos.rz)
                #current_pos.x += inverted.x
                #current_pos.y += inverted.y

        return result

    


    def disconnect(self):
        self.robot.disconnect()

    def _set_gripper(self, state: bool):
        gripper_report = Cobot_Operation_Report()


        gripper_job = Cobot_Job.disengage_gripper
        if state: gripper_job = Cobot_Job.engage_gripper

        result = self.robot.write_general_output_bit(self.gripper_io_group, self.gripper_io_bit, state)

        gripper_report.add(gripper_job,result)

        return gripper_report

    async def _assert_sensor(
        self,
        report: Cobot_Operation_Report,
        sensor: Cobot_Sensor,
        expected: bool,
        ) -> bool:

        actual = False
        
        for read_attempt in range (SENSOR_READ_ATTEMPTS):

            last_result = self.robot.read_general_output_bit(sensor.group, sensor.bit)

            actual = last_result.payload
            
            if(actual == expected):
                break

            await asyncio.sleep(SENSOR_READ_FREQUENCY)
        
        report.add(Cobot_Job.sensor_read,last_result)
        report.add_sensor_assert(sensor,expected,actual)

        return actual == expected
    
    async def _read_sensor(self,report: Cobot_Operation_Report,sensor: Cobot_Sensor) -> bool:

        last_result = self.robot.read_general_output_bit(sensor.group, sensor.bit)
        report.add(Cobot_Job.sensor_read,last_result)
        return last_result.payload
        
    



    



