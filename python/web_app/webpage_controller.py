from cobot_application.shared_data import P2P_command, sender_ID


class Webpage_Controller:
    def __init__(self, network):
        self.network = network

    async def send_command(self, receiver_ID, message, value=None):
        print("WebpageController received command request", flush=True)
        print("Receiver:", receiver_ID, flush=True)
        print("Message:", message, flush=True)
        print("Value:", value, flush=True)

        command = P2P_command(
            sender_ID=sender_ID.web_id,
            receiver_ID=receiver_ID,
            message=message,
            value=value
        )

        if receiver_ID == sender_ID.cobot_id:
            print("Putting command into cobot queue", flush=True)
            await self.network.cobot_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to cobot"}

        if receiver_ID == sender_ID.vision_id:
            print("Putting command into vision queue", flush=True)
            await self.network.vision_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to vision system"}

        if receiver_ID == sender_ID.camera_id:
            print("Putting command into camera queue", flush=True)
            await self.network.camera_commands.put(command)
            return {"ok": True, "message": f"Sent '{message}' to camera"}

        return {"ok": False, "message": "Unknown receiver ID"}