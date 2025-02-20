import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()

        # Subscribe to the color and depth image topics
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback_color, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.image_callback_depth, 10)

        # Variables to store the latest images
        self.color_image = None
        self.depth_image = None

    def image_callback_color(self, msg):
        # Convert ROS Image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the color image
        if self.color_image is not None:
            self.display_color_image()

    def image_callback_depth(self, msg):
        # Convert ROS Image message to NumPy array
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Display the depth image
        if self.depth_image is not None:
            self.display_depth_image()

    def display_color_image(self):
        # Ponto central para calcular distância
        x, y = 320, 240
        if self.depth_image is not None:
            # Calcular a distância no ponto (x, y)
            distance = self.depth_image[y, x] / 1000.0  # Conversão de mm para metros
            print(f"Distância no ponto ({x}, {y}): {distance:.3f} m")

            # Marcar o ponto na imagem RGB
            cv2.circle(self.color_image, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(self.color_image, f"{distance:.2f}m", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Mostrar a imagem colorida
        cv2.imshow("Color Image", self.color_image)
        cv2.waitKey(1)

    def display_depth_image(self):
        # Normalizar a imagem de profundidade para exibição
        depth_image_normalized = np.uint8(self.depth_image / np.max(self.depth_image) * 255)
        # Criar mapa de profundidade colorido
        depth_colormap = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # Mostrar a imagem de profundidade
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1)

def main():
    # Initialize the ROS2 node
    rclpy.init()

    # Cria ImageViewerNode
    image_viewer = ImageViewerNode()

    # Mantém o coódigo rodando
    rclpy.spin(image_viewer)

    # Destroy the node when it's finished
    image_viewer.destroy_node()
    rclpy.shutdown()