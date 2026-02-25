import torch
import numpy as np

class DepthEstimator:
    def __init__(self, model_type="MiDaS_small", device="cpu"):
        self.device = torch.device(device)

        self.model = torch.hub.load("intel-isl/MiDaS", model_type)
        self.model.to(self.device)
        self.model.eval()

        transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = transforms.small_transform

    def predict(self, frame):
        """
        frame: BGR image from OpenCV
        returns: depth_map (numpy 2D array)
        """
        input_batch = self.transform(frame).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=frame.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth = prediction.cpu().numpy()

        # Normalize to avoid numeric explosion
        depth = depth / np.max(depth)

        return depth
