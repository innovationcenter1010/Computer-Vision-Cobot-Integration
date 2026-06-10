import asyncio
import uvicorn

from cobot_application.shared_data import Py_network,Config

#from cobot_application.camera.camera_controller import Camera_Controller
from cobot_application.cobot.cobot_controller import Cobot_Controller
#from cobot_application.vision_system.vision_controller import Vision_Controller
from web_app.web_server import create_web_app

async def main():
    print("Hello, Cobot!")

    #application network is a type stored accessible to all modules with command queues for each of them
    application_network = Py_network()
    application_configurations = Config()

    #initialize the controller objects with access to the network and configuration objects
    cobot = Cobot_Controller(application_network,application_configurations)
    #vision = Vision_Controller(application_network,application_configurations)
    #camera = Camera_Controller(application_network,application_configurations)
    
    webpage_app = create_web_app(
        network=application_network,
        configurations=application_configurations
    )

    webpage_server = uvicorn.Server(
        uvicorn.Config(
            webpage_app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )
    )


    #create tasks for each of the fourn packge loops 
    tasks = [

       asyncio.create_task(webpage_server.serve()),     
       #asyncio.create_task(camera.loop()),
       #asyncio.create_task(vision.loop()),
       #asyncio.create_task(robot_loop(robot_queue, state)),
       asyncio.create_task(cobot.loop()),
    ]


    await asyncio.gather(*tasks)


asyncio.run(main())
