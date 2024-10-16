import cv2
import torch
from torch.cuda.amp import autocast
from depth_anything.dpt import DepthAnything

device = "cuda"
print("Using device:", device)

# Load the DepthAnything model
encoder = 'vits'
print(f"Loading DepthAnything model with encoder: {encoder}...")
depth_anything = DepthAnything.from_pretrained(f'LiheYoung/depth_anything_{encoder}14')
print("Model loaded successfully.")

depth_anything = depth_anything.to(device)
depth_anything = depth_anything.eval()

def preprocess_image(image_path):
    print(f"Reading image from: {image_path}")
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image not found.")
        return None
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0
    img = cv2.resize(img, (210, 210))
    img = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0)
    print("Image preprocessed and converted to tensor.")
    return img.to(device)  # Move the image tensor to CUDA

image_path = '/home/craters/craters-inference/utils/2024-07-27/img2024-07-27 18:36:46.893622.png'
image = preprocess_image(image_path)

if image is not None:
    print("Running inference...")
    with torch.no_grad():
        with autocast():
            depth = depth_anything(image)
    print("Inference completed.")

    depth = depth.cpu().numpy()
    print("Depth map generated and moved to CPU.")
else:
    print("Inference could not be performed due to image loading error.")

