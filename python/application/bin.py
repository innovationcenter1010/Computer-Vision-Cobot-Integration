import config
from layers import OrderedLayer


class Bin:
    NEXT_LAYER_READY = "NEXT_LAYER_READY"
    CONTINUE_CURRENT_LAYER = "CONTINUE_CURRENT_LAYER"
    BIN_COMPLETE = "BIN_COMPLETE"

    def __init__(self):
        self.layer_depths = [
            getattr(config, f"LAYER_Z{i}") for i in range(1, 8)
        ]

        self.depth_index = 0
        self.depth = self.layer_depths[self.depth_index]

        self.layer = None

        # Number of cans detected when the current layer was first loaded.
        # This is used later for the threshold check.
        self.current_layer_reference_size = None

        self.bin_complete = False

    def _load_layer_at_depth(self, layer_path, depth):
        """
        Loads a new JSON scan file and assigns the given z-depth to each can.
        """
        layer = OrderedLayer(
            layer_path,
            config.BIN_IMAGE_CORNERS,
            config.BIN_REAL_WORLD_CORNERS
        )

        for can in layer.order:
            can.real_z = depth

        return layer

    def add_layer(self, layer_path):
        """
        Loads the first visible layer of a bin.

        Important:
        - This does not check the threshold.
        - This does not advance the z-depth.
        - For a new bin, layer 1 is accepted immediately.
        """
        self.layer = self._load_layer_at_depth(layer_path, self.depth)

        if self.layer.size > 0:
            self.current_layer_reference_size = self.layer.size

        return self.layer

    def check_layer_complete(self, layer_path):
        """
        Called after the cobot thinks it finished the current layer.

        It loads a completely new JSON scan file and decides whether to:
        - keep picking the current layer,
        - advance to the next layer,
        - or complete the bin after layer 7.
        """
        detected_layer = self._load_layer_at_depth(layer_path, self.depth)
        detected_count = detected_layer.size

        # Layer 7 special rule:
        # The bin is only complete when layer 7 detects 0 cans.
        if self.depth_index == len(self.layer_depths) - 1:
            if detected_count == 0:
                self.bin_complete = True
                self.layer = None
                return Bin.BIN_COMPLETE

            self.layer = detected_layer
            return Bin.CONTINUE_CURRENT_LAYER

        if self.current_layer_reference_size is None:
            if detected_count > 0:
                self.layer = detected_layer
                self.current_layer_reference_size = detected_count

            return Bin.CONTINUE_CURRENT_LAYER

        threshold = config.LAYER_THRESHOLD * self.current_layer_reference_size

        if detected_count >= threshold:
            # Enough cans were detected, so the previous layer is considered complete.
            # Move down to the next z-depth.
            self.depth_index += 1
            self.depth = self.layer_depths[self.depth_index]

            # The detected cans are now treated as the next layer.
            for can in detected_layer.order:
                can.real_z = self.depth

            self.layer = detected_layer
            self.current_layer_reference_size = self.layer.size

            return Bin.NEXT_LAYER_READY

        # Too few cans were detected.
        # This means the cobot probably did not actually finish the current layer.
        # Stay at the same z-depth and keep picking.
        self.layer = detected_layer
        return Bin.CONTINUE_CURRENT_LAYER

    def pickup(self):
        """
        Returns the next can to pick from the current layer.
        Returns None if no cans remain.
        """
        if not self.layer or not self.layer.order:
            return None

        can = self.layer.order.pop(0)
        self.layer.size = len(self.layer.order)

        return can