import layers as layers
from pathing import order_layer

new_layer = layers.Layer("sample_workflow/sample_data.JSON")

new_layer = order_layer(new_layer)

print("Image dimensions: " + str(new_layer.image_width) + "x" + str(new_layer.image_height))
print("Number of cans: " + str(new_layer.size))
print("Cans in order:")
for can in new_layer.cans:
    print("Class: " + can.can_class + ", Class ID: " + str(can.class_id) + ", Confidence: " + str(can.confidence) + ", Center: (" + str(can.center_x) + ", " + str(can.center_y) + ")")