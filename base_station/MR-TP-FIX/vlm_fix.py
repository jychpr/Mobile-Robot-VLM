import os
import time
from PIL import Image
import moondream as md

# Initialize the VLM model
model = md.vl(model="models/moondream-0_5b-int8.mf")
# model = md.vl(model="models/moondream-2b-int8.mf")

# Folder to monitor for new images
image_directory = "received_images"
processed_images = set()  # To keep track of already processed images
wait_time = 1  # Time to wait between checks (in seconds)

print("Starting image monitoring...")

elapsed_time = 0  # To keep track of the elapsed time while waiting

while True:
    # Get the list of all image files in the directory
    all_files = set(os.listdir(image_directory))
    image_files = {f for f in all_files if f.endswith(('.jpg', '.png', '.jpeg'))}
    
    # Identify new images
    new_images = image_files - processed_images

    if new_images:
        elapsed_time = 0  # Reset the elapsed time when a new image is found
        for image_file in new_images:
            try:
                image_path = os.path.join(image_directory, image_file)
                print(f"Processing new image: {image_file}")
                
                # Load and rotate the image
                image = Image.open(image_path)
                image = image.rotate(180)
                
                # Process the rotated image
                encoded_image = model.encode_image(image)

                # Ask questions and get the answer
                answer = model.query(
                    encoded_image,
                    "Describe your point of view and the objects in front of you might be obstacles, list the objects"
                )["answer"]

                print("Answer:", answer)
                
                # Mark the image as processed
                processed_images.add(image_file)

            except Exception as e:
                print(f"Error processing image {image_file}: {e}")
    else:
        elapsed_time += wait_time
        print(f"Waiting {elapsed_time} seconds for new images...")
        time.sleep(wait_time)
