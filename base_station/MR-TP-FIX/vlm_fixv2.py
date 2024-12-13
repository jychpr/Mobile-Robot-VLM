import os
import time
from PIL import Image
import moondream as md

# Initialize the VLM model
model = md.vl(model="models/moondream-0_5b-int8.mf")
# model = md.vl(model="models/moondream-2b-int8.mf")

# Folder to monitor for new images
image_directory = "received_images"
processed_image = None  # Keep track of the last processed image
wait_time = 0.5  # Time to wait between checks (in seconds)

print("Starting image monitoring...")

while True:
    try:
        # Get the list of all image files in the directory, sorted by modification time
        image_files = [
            f for f in os.listdir(image_directory)
            if f.endswith(('.jpg', '.png', '.jpeg'))
        ]
        
        if image_files:
            # Sort files by modification time (newest last)
            image_files = sorted(
                image_files,
                key=lambda x: os.path.getmtime(os.path.join(image_directory, x))
            )
            latest_image = image_files[-1]  # Get the newest file

            # Process the newest image only if it hasn't been processed yet
            if latest_image != processed_image:
                image_path = os.path.join(image_directory, latest_image)
                print(f"Processing latest image: {latest_image}")

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

                print(f"Answer for {latest_image}: {answer}")

                # Mark the latest image as processed
                processed_image = latest_image
            else:
                print(f"No new images to process. Waiting {wait_time} seconds...")
        else:
            print(f"No images found. Waiting {wait_time} seconds...")

        time.sleep(wait_time)

    except Exception as e:
        print(f"Error: {e}")
        time.sleep(wait_time)
