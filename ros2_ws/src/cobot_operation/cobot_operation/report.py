"""Operation report — pure Python, mirrors the legacy Cobot_Operation_Report
(text format kept so logs read the same) and converts to the JobResult /
SensorAssert messages carried in PickCan.Result."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

STATUS_TRANSPORT = -1  # timeout / rejected / server gone — never a controller code


@dataclass
class JobResult:
    job: str
    ok: bool
    status_code: int = 0
    message: str = ""
    payload: str = ""

    @staticmethod
    def transport(job: str, message: str, payload: str = "") -> "JobResult":
        return JobResult(job, False, STATUS_TRANSPORT, message, payload)


@dataclass
class SensorAssert:
    name: str
    expected: bool
    actual: bool
    passed: bool


@dataclass
class OperationReport:
    title: str = "none"
    jobs: List[JobResult] = field(default_factory=list)
    sensor_asserts: List[SensorAssert] = field(default_factory=list)
    failures: List[str] = field(default_factory=list)  # explicit failure notes

    # ---- building ----

    def add(self, job: JobResult) -> JobResult:
        self.jobs.append(job)
        if not job.ok:
            self.failures.append(f"{job.job}: {job.message}")
        return job

    def add_sensor_assert(self, name: str, expected: bool, actual: bool) -> bool:
        passed = expected == actual
        self.sensor_asserts.append(SensorAssert(name, expected, actual, passed))
        if not passed:
            self.failures.append(f"sensor {name}: expected {expected}, got {actual}")
        return passed

    def mark_failed(self, note: str) -> None:
        self.failures.append(note)

    # ---- querying ----

    @property
    def overall_ok(self) -> bool:
        return not self.failures

    def first_failure(self) -> Optional[str]:
        return self.failures[0] if self.failures else None

    # ---- text (same shape as the legacy report) ----

    def __str__(self) -> str:
        lines = ["=" * 60, "COBOT OPERATION REPORT" if self.title == "none" else self.title, "=" * 60]
        lines.append(f"Overall Status : {'job_success' if self.overall_ok else 'job_failure'}")
        lines.append(f"Jobs Executed  : {len(self.jobs)}")
        lines.append(f"Sensor Asserts : {len(self.sensor_asserts)}")
        lines.append("")
        if self.jobs:
            lines += ["JOB RESULTS", "-" * 60]
            for idx, j in enumerate(self.jobs, start=1):
                lines.append(
                    f"[{idx:02d}] {j.job:<25} ok={j.ok} code={j.status_code} "
                    f"payload={j.payload} {j.message}"
                )
            lines.append("")
        if self.sensor_asserts:
            lines += ["SENSOR ASSERTS", "-" * 60]
            for idx, a in enumerate(self.sensor_asserts, start=1):
                lines.append(
                    f"[{idx:02d}] {a.name:<20} expected={a.expected} actual={a.actual} "
                    f"[{'PASS' if a.passed else 'FAIL'}]"
                )
            lines.append("")
        if self.failures:
            lines += ["FAILURES", "-" * 60]
            lines += [f"- {f}" for f in self.failures]
            lines.append("")
        return "\n".join(lines)
