import asyncio

from cobot_application.shared_data import Py_network,Config

#from cobot_application.camera.camera_controller import Camera_Controller
#from cobot_application.cobot.cobot_controller import Cobot_Controller
from cobot_application.vision_system.vision_controller import Vision_Controller
from cobot_application.web_app.webpage_controller import Webpage_Controller

async def main():
    print("Hello, Cobot!")

    #application network is a type stored accessible to all modules with command queues for each of them
    application_network = Py_network()
    application_configurations = Config()

    #initialize the controller objects with access to the network and configuration objects
    #cobot = Cobot_Controller(application_network,application_configurations)
    vision = Vision_Controller(application_network,application_configurations)
    #camera = Camera_Controller(application_network,application_configurations)
    webpage = Webpage_Controller(application_network, application_configurations)
    

    #create tasks for each of the fourn packge loops 
    tasks = [
       asyncio.create_task(webpage.loop()),     
       #asyncio.create_task(camera.loop()),
       asyncio.create_task(vision.loop()),
       #asyncio.create_task(cobot.loop()),
    ]


    await asyncio.gather(*tasks)


asyncio.run(main())
