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

        # Variáveis para armazenar as imagens mais recentes
        self.color_image = None
        self.depth_image = None

        #ponto inicial
        self.selected_point = (320,240)
        
        # Criar a janela e associar o callback do mouse
        cv2.namedWindow("Color Image")
        cv2.setMouseCallback("Color Image", self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):
    #Callback para capturar clique do mouse
        if event == cv2.EVENT_LBUTTONDOWN:  # Clique com o botão esquerdo
            self.selected_point = (x, y)
            self.get_logger().info(f"Novo ponto selecionado: {self.selected_point}")

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
        x, y = self.selected_point #ponto selecionado pelo usuário com o mouse
        if self.depth_image is not None:
            # Calcular a distância no ponto (x, y) e verificar se as coordenadas estão dentro dos limites do shape(quadro da câmera)
            if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0] :
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
    rclpy.init()# Initialize the ROS2 node
    image_viewer = ImageViewerNode()# Cria ImageViewerNode
    rclpy.spin(image_viewer)# Mantém o coódigo rodando
    image_viewer.destroy_node()# Destroy the node when it's finished
    rclpy.shutdown()