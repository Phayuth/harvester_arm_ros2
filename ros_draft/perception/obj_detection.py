import cv2
import numpy as np
import rclpy
import ultralytics
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor


class ImageDetection(Node):

    def __init__(self):

        super().__init__('image_detection')
        self.detectionModel = ultralytics.YOLO('./weight/yolov8x-seg.pt')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.masked_image, 10)
        self.publisher = self.create_publisher(Image, '/detection', 10)
        self.br = CvBridge()

    def bounding_box_image(self, data):
        imgBGR = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        results = self.detectionModel(imgRGB, stream=True, conf=0.5, verbose=True)
        for r in results:
            boxes = r.boxes
            for eachbox in boxes:
                x1, y1, x2, y2 = eachbox.xyxy[0]
                confident = eachbox.conf[0]
                classes_number = int(eachbox.cls[0].item())
                classes_name = self.detectionModel.names[classes_number]
                if classes_name == "apple":
                    cv2.putText(imgRGB,
                                text=classes_name + str(round(confident.item(), 2)),
                                org=(int(x1), int(y1 + (y2-y1) / 2)),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=1,
                                thickness=2,
                                color=(255, 0, 255))
                    cv2.rectangle(imgRGB, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 3)
        imgBGR = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2BGR)
        imgData = self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8")
        self.publisher.publish(imgData)

    def masked_image(self, data):
        imgBGR = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        imgShow = np.zeros_like(imgRGB)
        imgMask = np.zeros((imgShow.shape[0], imgShow.shape[1]))
        results = self.detectionModel(imgRGB, stream=True, conf=0.5, verbose=True)
        for r in results:
            boxes = r.boxes
            masks = r.masks
            for bi, box in enumerate(boxes):
                classes_number = int(box.cls[0].item())
                classes_name = self.detectionModel.names[classes_number]
                if classes_name == "apple":
                    appleMask = masks.data[bi, :, :].cpu().numpy()
                    appleMask = appleMask.reshape(appleMask.shape[0], appleMask.shape[1])
                    imgMask = imgMask + appleMask
        imgshow = np.where(imgMask[..., None], imgRGB, 0)
        imgBGR = cv2.cvtColor(imgshow, cv2.COLOR_RGB2BGR)
        imgData = self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8")
        self.publisher.publish(imgData)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageDetection()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()