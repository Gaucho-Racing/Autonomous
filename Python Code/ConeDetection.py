import torch
from PIL import Image
from torchvision.datasets import CocoDetection, ImageFolder
import torchvision.transforms as T

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

class ConeDataset():
    def __init__(self, data_dir, transform=None):
        self.data = ImageFolder(data_dir, transform=transform)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx]

    @property
    def classes(self):
        return self.data.classes

#dataset = ConeDataset(data_dir=...)