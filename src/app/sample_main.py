import layers as layers

newLayer = layers.Layer("sample_workflow/sample_data.JSON")

print("Image width: ", newLayer.image_width) 
print("Image height: ", newLayer.image_height)
print("Number of cans: ", len(newLayer.cans))
