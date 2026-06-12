import asyncio
from cobot_application.cobot import motoman_cpp

async def run_cobot_calibration(cobot, configurations=None, value=None):
    print("CALIBRATION STARTED", flush=True)

    p1 = motoman_cpp.Position(900,-600,500,180,0,0)

    await cobot.goto(p1)

    await asyncio.sleep(0.2)
    
    print("CALIBRATION FINISHED", flush=True)

    return {
        "ok": True,
        "message": "Calibration complete"
    }

