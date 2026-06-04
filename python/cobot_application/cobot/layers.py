import json


class Can:
    def __init__(self, can_data):
        self.can_class = can_data["class"]
        self.class_id = can_data["class_id"]
        self.confidence = can_data["confidence"]
        self.corner_x = can_data["x"]
        self.corner_y = can_data["y"]
        self.width = can_data["width"]
        self.height = can_data["height"]

        self.center_x = self.corner_x + self.width // 2
        self.center_y = self.corner_y + self.height // 2


        self.real_x = None # Real-world x-coordinate (to be calculated later)
        self.real_y = None # Real-world y-coordinate (to be calculated later)
        self.real_z = None # Real-world z-coordinate (to be calculated later)
        self.real_rz = None # Real-world rotation around z-axis (to be calculated later)


class Layer:
    def __init__(self, json_file_path):
        with open(json_file_path, "r") as file:
            data = json.load(file)

        predictions_root = data[0]["predictions"]

        self.image_width = predictions_root["image"]["width"]
        self.image_height = predictions_root["image"]["height"]

        self.cans = []
        predictions = predictions_root["predictions"]

        for prediction in predictions:
            self.cans.append(Can(prediction))

        self.size = len(self.cans)

    def peek_front(self):
        if not self.cans:
            return None
        return self.cans[0]

    def pop_front(self):
        if not self.cans:
            return None
        can = self.cans.pop(0)
        self.size = len(self.cans)
        return can


class OrderedLayer(Layer):
    """
    Loads a JSON file, orders the cans in snake-pattern order,
    and translates each can into real-world coordinates.

    After creating an OrderedLayer, each can in layer.order will have:
        can.real_x
        can.real_y
    """

    def __init__(self, json_file_path, image_corners, real_world_corners):
        # Step 1: Load cans from JSON
        super().__init__(json_file_path)

        # Step 2: Order cans in snake-pattern (local import avoids circular dependency)
        from python.cobot.application.pathing import order_layer
        order_layer(self)

        # Step 3: Translate pixel coordinates to real-world coordinates
        from python.cobot.application.homography import HomographyTranslator
        translator = HomographyTranslator(image_corners, real_world_corners)
        translator.translate_layer(self)