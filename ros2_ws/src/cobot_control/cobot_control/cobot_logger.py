from pathlib import Path
from datetime import datetime
from cobot_application.cobot.cobot import Cobot_Operation_Report


def log_report(report: Cobot_Operation_Report,filename: str = "cobot_log.txt") -> Path:

    filepath = Path(filename)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 80 + "\n")
        f.write(
            f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        )
        f.write(f"Job Title: {report.job_title}\n")
        f.write(f"Overall Status: {report.overall_status.name}\n")
        f.write("=" * 80 + "\n\n")

        # Job Results
        for idx, (job, result) in enumerate(
            zip(report.jobs, report.results),
            start=1,
        ):
            f.write(
                f"[{idx:03d}] {job.name}\n"
            )

        # Sensor Assertions
        for idx, assertion in enumerate(
            report.sensor_asserts,
            start=1,
        ):
            f.write(
                f"[{idx:03d}] "
                f"{assertion.sensor.name} "
                f"expected={assertion.expected} "
                f"actual={assertion.actual} "
                f"passed={assertion.passed}\n"
            )

    return filepath