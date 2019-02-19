# Use this code as a sample to create a node capable of classifying the contents of an image

# For use in X-491, Spring 2019

import keras, tensorflow, numpy, pdb # Import Keras, Tensorflow, and NumPY (API calls, low-level deep learning software, and math package)
from keras.applications import mobilenet_v2 # A mobilenet is useful because our VMs are limited computationally -- this network is designed to run on a mobile device, like a cell phone
from keras.utils.vis_utils import plot_model # We use this to print the model description
from keras.preprocessing.image import load_img, img_to_array
from keras.applications.mobilenet_v2 import preprocess_input, decode_predictions

model = mobilenet_v2.MobileNetV2(weights='imagenet') # The first time you run this script, Google's pretrained network weights are downloaded. This may take a few minutes so don't worry if it seems to "stall out"
 
print(model.summary()) # Print a summary of the model's architecture
plot_model(model, to_file='mobilenet.png') # This creates a .png showing the model's architecture
 
#pdb.set_trace() # You may want to use PDB to step through code here

## TODO: write preprocessing here - you can use OpenCV or keras.preprocessing
# The model expects images of a particular size here - you can use opencv, or image=load_img(filehandle, target_size=(X,Y))
# The image size is called out in the first layer of the model (X,Y,3) for the (X,Y,RGB) dimensions
# If you don't resize, the model will work poorly or not at all

# Once your image is resized, convert it to an array
image = img_to_array(image)

# The model expects data in a particular format: (Samples, rows, columns, channels)
# We only have one file, so use reshape to add the sample dimension
# image = image.reshape((A, X, Y, 3))

# Many models require color resampling, e.g. subtracting the average R, G, and B values from the image before classification
# Keras handles this automatically using image = preprocess_input(image)

# Finally, make a prediction
model_output = model.predict(image)

# The output must be decoded
possible_labels = decode_predictions(model_output)

# Up to you: select the highest probability prediction
real_label = possible_labels[x][y]

label_text = real_label[x]
label_probability = real_label[y]