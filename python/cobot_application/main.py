import asyncio
from cobot_application.shared_data import Py_network,Config

from cobot_application.camera.camera_controller import Camera_Controller
from cobot_application.cobot.cobot_controller import Cobot_Controller
from cobot_application.vision_system.vision_controller import Vision_Controller
#from webpage_controller import Webpage_Controller


async def main():
    print("Hello, Cobot!")


    application_network = Py_network()
    application_configurations = Config()

    cobot = Cobot_Controller(application_network,application_configurations)
    vision = Vision_Controller(application_network,application_configurations)
    camera = Camera_Controller(application_network,application_configurations)


    tasks = [
        asyncio.create_task(
            camera.loop()
        ),

        asyncio.create_task(
            vision.loop()
        ),

        #asyncio.create_task(
        #    robot_loop(robot_queue, state)
        #),

        asyncio.create_task(
            cobot.loop()
        ),
    ]

    await asyncio.gather(*tasks)


asyncio.run(main())
