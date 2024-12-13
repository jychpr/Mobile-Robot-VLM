import moondream as md
from PIL import Image

# Initialize with local model path. Can also read .mf.gz files, but we recommend decompressing
# up-front to avoid decompression overhead every time the model is initialized.
model = md.vl(model="models/moondream-0_5b-int8.mf")

# Load and process image
image = Image.open("camimage.jpg")
encoded_image = model.encode_image(image)

# Ask questions
answer = model.query(encoded_image, "What is the objects in the image?")["answer"]
print("Answer:", answer)


