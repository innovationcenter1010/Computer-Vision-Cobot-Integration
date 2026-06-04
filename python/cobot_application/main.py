import asyncio
from shared_data import Py_network,Config

#from camera_controller import camera_controller
from cobot_controller import Cobot_Controller
#from yolo_controller import Yolo_Controller
#from webpage_controller import Webpage_Controller


async def main():
    print("Hello, Cobot!")

    application_network = Py_network()
    application_configurations = Config()

    cobot = Cobot_Controller(application_network,application_configurations)

    tasks = [
        #asyncio.create_task(
        #    camera_loop(camera_queue, state)
        #),

        #asyncio.create_task(
        #    cv_loop(camera_queue, robot_queue, state)
        #),

        #asyncio.create_task(
        #    robot_loop(robot_queue, state)
        #),

        asyncio.create_task(
            cobot.loop()
        ),
    ]

    await asyncio.gather(*tasks)


asyncio.run(main())
