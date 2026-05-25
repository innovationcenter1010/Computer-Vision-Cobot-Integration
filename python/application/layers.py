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

        self.center_x = self.corner_x + self.width // 2 # Calculate center x-coordinate
        self.center_y = self.corner_y + self.height // 2 # Calculate center y-coordinate

class Layer:
    def __init__(self, json_file_path):
        # Load JSON file
        with open(json_file_path, "r") as file:
            data = json.load(file)

        # Access root object
        predictions_root = data[0]["predictions"]

        # Save image dimensions
        self.image_width = predictions_root["image"]["width"]
        self.image_height = predictions_root["image"]["height"]

        # Create array of Can objects
        self.cans = []

        predictions = predictions_root["predictions"]

        #go through all cans in the predictions and create Can objects
        i = 0
        while i < len(predictions):
            can_obj = Can(predictions[i])
            self.cans.append(can_obj)
            i += 1

        #store size of the layer (number of cans)
        self.size = len(self.cans)


    def peek_front(self):
        """
        Returns the first Can in the list without removing it.
        """
        if not self.cans:
            return None
        return self.cans[0]

    def pop_front(self):
        """
        Removes and returns the first Can in the list.
        """
        if not self.cans:
            return None
        can = self.cans.pop(0)
        self.size = len(self.cans)
        return can
    

    def path(self):
        import python.app.pathing as pathing
        
        self = pathing.order_layer(self)


