import asyncio
import time
import motoman_cpp
from cobot_application.cobot import cobot

async def run_cobot_calibration(cobot:cobot):
    print("CALIBRATION STARTED", flush=True)

    p1 = [motoman_cpp.Position(900,-600,600,180,0,0)]

    await cobot.goto(p1)

    asyncio.sleep(0.2)

