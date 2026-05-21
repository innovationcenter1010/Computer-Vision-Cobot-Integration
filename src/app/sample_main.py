from layers import Layer

# Sample main.py to test the Layer class and its pathing method.
new_layer = Layer("sample_workflow/sample_data.JSON")

new_layer.path()

print("Image dimensions: " + str(new_layer.image_width) + "x" + str(new_layer.image_height))
print("Number of cans: " + str(new_layer.size))
print("Cans in order:")
for can in new_layer.cans:
    print("Class: " + can.can_class + ", Class ID: " + str(can.class_id) + ", Confidence: " + str(can.confidence) + ", Center: (" + str(can.center_x) + ", " + str(can.center_y) + ")")