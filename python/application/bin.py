import layers

class Bin:
    def __init__(self,depth):
        self.depth = depth
        self.layer = None
        self.pick_path = None

    def add_layer(self, layer_path):
        """
        Adds a Layer object to the Bin.

        layer_path: the file path to the JSON file containing the layer data.
        """

        self.layer = layers.Layer(layer_path)

        """ 
        TODO:run the layer through the pathing algorithm and homography to get the pick path for this layer.
        self.pickpath = pathing.order_layer(self.layer)
        """


    def pickup(self):
  
        if self.layer.cans:
            # .pop(0) picks up from the front, .pop() would pick up from the back
        
            return self.pick_path.pop(0)
        else:
            return None