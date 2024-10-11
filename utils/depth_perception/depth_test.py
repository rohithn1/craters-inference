device = "cuda"

from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import torch
import numpy as np
from PIL import Image
import requests
import matplotlib.pyplot as plt
import time

image_processor = AutoImageProcessor.from_pretrained("LiheYoung/depth-anything-small-hf") # 99.2 mb
model = AutoModelForDepthEstimation.from_pretrained("LiheYoung/depth-anything-small-hf").to(device)


