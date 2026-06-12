import asyncio
import time
import motoman_cpp
from cobot_application.cobot import cobot

async def run_cobot_calibration(cobot:cobot):
    print("CALIBRATION STARTED", flush=True)

    p1 = [motoman_cpp.Position(900,-600,500,180,0,0)]
    p2 = [motoman_cpp.Position(900,-600,700,180,0,90)]
    p3 = [motoman_cpp.Position(900,-600,500,180,0,270)]

    await cobot.goto(p1,p2,p3)

    asyncio.sleep(0.2)

