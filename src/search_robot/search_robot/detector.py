import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge

from search_interfaces.msg import DetectedCube, DetectedCubeArray


class ArucoDetector(Node):
    def __init__(
        self,
        marker_size: float = 0.0701,   # rzeczywisty rozmiar markera Aruco 
        max_distance: float = 4.0,     # maksymalny dystans detekcji
        outlier_threshold: float = 0.8,# prog odrzucania skokow pomiaru
        min_confirm_frames: int = 3,   # liczba obrazów do potwierdzenia kostki
    ):
        super().__init__('aruco_detector')
        self.marker_size = marker_size
        self.max_distance = max_distance
        self.outlier_threshold = outlier_threshold
        self.min_confirm_frames = min_confirm_frames
        self.bridge = CvBridge()

        # Parametry wewnetrzne kamery (wypelniane z /camera/camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Konfiguracja ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Bufor transformacji przestrzennych TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detected_cubes = {}

        # Publisher wykrytych kostek
        self.cubes_pub = self.create_publisher(DetectedCubeArray, '/detected_cubes', 10)

        # Subskrypcje obrazu i parametrow kamery
        self.create_subscription(CameraInfo, '/camera/camera_info', self._camera_info_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_callback, 10)

    def _camera_info_callback(self, msg: CameraInfo):
        # Pobranie macierzy kamery na poczatku
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def _image_callback(self, msg: Image):
        # Czekanie na parametry kamery
        if self.camera_matrix is None:
            return

        # Konwersja do OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # Detekcja znacznikow na obrazie w skali szarosci
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            return

        # Estymacja pozycji wzgledem kamery
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            # Ograniczenie do ID od 0 do 4
            if mid < 0 or mid > 4:
                continue

            tvec = tvecs[i][0]
            detection_distance = float(np.linalg.norm(tvec))

            # Odrzucenie obserwacji powyżej dopuszczalnego dystansu
            if detection_distance > self.max_distance:
                continue

            # Przygotowanie punktu w ukladzie optycznym kamery
            point_camera = PointStamped()
            point_camera.header.frame_id = 'camera_rgb_optical_frame'
            point_camera.header.stamp = msg.header.stamp
            point_camera.point.x = float(tvec[0])
            point_camera.point.y = float(tvec[1])
            point_camera.point.z = float(tvec[2])

            # Transformacja punktu z kamery do ukladu map
            try:
                try:
                    point_map = self.tf_buffer.transform(point_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.3))
                except Exception:
                    point_camera.header.stamp = rclpy.time.Time().to_msg()
                    point_map = self.tf_buffer.transform(point_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.3))
                coords = (point_map.point.x, point_map.point.y, point_map.point.z)

                if mid not in self.detected_cubes:
                    self.detected_cubes[mid] = []

                # Odrzucenie skokow pomiaru
                if self.detected_cubes[mid]:
                    prev_coords = self.detected_cubes[mid][-1]
                    dist_to_prev = np.hypot(coords[0] - prev_coords[0], coords[1] - prev_coords[1])
                    if dist_to_prev > self.outlier_threshold:
                        continue

                # Zapisanie poprawnego pomiaru
                self.detected_cubes[mid].append(coords)

                # Informacja w konsoli po osiagnieciu progu potwierdzenia
                if len(self.detected_cubes[mid]) == self.min_confirm_frames:
                    self.get_logger().info(f"Znaleziono kostke o ID={mid}: x={coords[0]:.2f}, y={coords[1]:.2f}")

                # Publikacja potwierdzonych kostek na topicu
                self._publish_cubes()
            except Exception:
                pass

    def _publish_cubes(self):
        msg = DetectedCubeArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for mid, samples in self.detected_cubes.items():
            if len(samples) >= self.min_confirm_frames:
                arr = np.array(samples)
                median_point = np.median(arr, axis=0)
                cube = DetectedCube()
                cube.id = mid
                cube.position.x = float(median_point[0])
                cube.position.y = float(median_point[1])
                cube.position.z = float(median_point[2])
                cube.observation_count = len(samples)
                msg.cubes.append(cube)
        self.cubes_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
