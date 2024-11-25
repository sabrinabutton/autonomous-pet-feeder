"""
This script recieves bytes of an image from an Arduino on a Flask server, and then runs it through a Cat/Dog classification CNN. It returns whether a cat or dog or neither was seen,
This enables deep-learning computer vision on a project with a low-processing power arduino by offloading image storage and computational load to a remote PC (on which you are running this Flask script)
"""
import os
from flask import Flask, request
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import io
from inference_sdk import InferenceHTTPClient

app = Flask(__name__)

# Directory to temporarily store image chunks
TEMP_DIR = "image_chunks"
os.makedirs(TEMP_DIR, exist_ok=True)

# Initialize variables to store received image
image_data = b""
image_complete = False

@app.route('/upload', methods=['POST'])
def upload():
    
    print("Received image chunk")
    global image_data, image_complete
    
    # Reset global variables for next image
    
    image_data = b""
    image_complete = False
    print("image data at reset is", image_data)
    
    # Read incoming binary data
    chunk = request.get_data()
    
    print("Chunk size:", len(chunk))

    # Check for EOF (End of File)
    if b"EOF" in chunk:
        # Remove EOF flag and anything after it
        chunk = chunk[:chunk.index(b"EOF")]
        image_complete = True

    

    # Append the chunk to image data
    image_data += chunk

    print("Total image size:", len(image_data))

    if image_complete:
        # Save and process the image
        return process_image()

    return "Chunk received", 200

def process_image():
    global image_data, image_complete

    try:
        # Use PIL to open the image directly from bytes
        image = Image.open(io.BytesIO(image_data))

        # Generate a random image name, append jpg extension
        image_name = "image" + str(np.random.randint(100000)) + ".jpg"

        # Write the image to a JPEG file
        image_path = os.path.join(TEMP_DIR, image_name)
        image.save(image_path)
        
        # Load the image and do computer vision processing
        CLIENT = InferenceHTTPClient(
            api_url="https://classify.roboflow.com",
            api_key="aEgr5ZO29cc9ixQtxHXi"
        )

        result = CLIENT.infer(os.path.join(TEMP_DIR, image_name), model_id="cat-dog-lf9qi/2")

        print(result)
        print(result["top"])
        
        
        
        if (result['predictions'][0]['confidence'] <= 0.8) and (result['predictions'][1]['confidence'] <= 0.8):
            print("No cats or dogs found")
            return "Output: NONE", 200
        else:
            print(result['top'])
            if(result['top'] == 'cats'):
                return "Output: CAT", 200
            else:
                return "Output: DOG", 200
        
    except Exception as e:
        print(e)
        
        # Reset everything
        image_data = b""
        image_complete = False
        
        return f"Output: FAIL", 500

    

    return "Image received and displayed", 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
