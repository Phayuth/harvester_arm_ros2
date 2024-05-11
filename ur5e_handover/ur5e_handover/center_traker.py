import numpy as np
import cv2
from collections import deque


class Center:

    def __init__(self):
        self.data_list_r = deque([])
        self.data_list_l = deque([])
        self.erodeKernel = np.ones((15, 15), np.uint8)

    def save_video(self, cap, name):
        """
        Initializes a video writer object to save video output.

        Parameters:
        - cap (cv2.VideoCapture): Video capture object that provides properties and video frames.
        - name (str): Name of the output video file.

        Returns:
        - cv2.VideoWriter: Initialized video writer object for output video.
        """

        w = round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)  # 카메라에 따라 값이 정상적, 비정상적

        fourcc = cv2.VideoWriter_fourcc(*"DIVX")

        out = cv2.VideoWriter(name, fourcc, fps, (w, h))

        return out

    def make_segmentation_masks(self, img_results, image_np):
        """
        Creates a segmentation mask for detected objects in an image.

        Parameters:
        - img_results (list): List of detection results, each containing masks and bounding boxes.
        - image_np (numpy.ndarray): The original image in numpy array format.

        Returns:
        - numpy.ndarray: Binary mask of detected objects.
        """

        mask_image = np.zeros(image_np.shape[:2], dtype=np.uint8)

        for result in img_results:
            if result.masks is None or result.boxes is None:
                continue

            for mask, box in zip(result.masks.xy, result.boxes):
                polygon = np.array(mask, dtype=np.int32)
                cv2.fillPoly(mask_image, [polygon], 255)

        cv2.erode(mask_image, self.erodeKernel, cv2.BORDER_REFLECT)
        return mask_image

    def draw_over_image_inplace(self, rgb_image, center):
        """
        Draws the original image and mask side-by-side with centroid highlighted.

        Parameters:
        - rgb_image (numpy.ndarray): Original RGB image.
        - center (tuple): Coordinates (x, y) of the centroid.

        Returns:
        - numpy.ndarray: Combined image showing the original and mask with centroid.
        """
        draw_cx, draw_cy = center
        if draw_cx != 0 and draw_cy != 0:
            cv2.circle(rgb_image, (draw_cx, draw_cy), 5, (255, 0, 255), -1)
            cv2.putText(rgb_image, text=f"centroid at {draw_cx}, {draw_cy}", org=(draw_cx, draw_cy), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1, color=(255, 0, 255))

    def draw_over_image(self, rgb_image, uv):
        draw_cx, draw_cy = uv
        if draw_cx != 0 and draw_cy != 0:
            cv2.circle(rgb_image, (draw_cx, draw_cy), 5, (255, 0, 255), -1)
            cv2.putText(rgb_image, text=f"{draw_cx}, {draw_cy}", org=(draw_cx, draw_cy), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1, color=(255, 0, 255))

    def compute_centroid_via_pixels_top(self, mask_image):
        """
        Computes the centroid of a mask image, adjusting the y-coordinate to capture a higher vertical position.

        Parameters:
        - mask_image (numpy.ndarray): Binary mask image where objects are marked.

        Returns:
        - tuple: Coordinates (x, y) of the adjusted centroid.
        """
        centroid_x = 0
        centroid_y = 0
        # 마스크 이미지가 전부 0인지 확인
        if np.any(mask_image):
            Y, X = np.nonzero(mask_image)
            if len(X) == 0 or len(Y) == 0:
                return None
            centroid_x = int(np.mean(X))
            centroid_y = int(np.min(Y) + 30)
        return (centroid_x, centroid_y)

    def mean_window(self, camera_location, data, length):
        if camera_location == "r":
            data_list = self.data_list_r
        if camera_location == "l":
            data_list = self.data_list_l
        else:
            print("camera location has only r or l")
        x = data[0]
        y = data[1]
        if x + y != 0 and (x or y is None):
            data_list.append([x, y])
            if len(data_list) > length - 1:
                data_list.popleft()
            x_sum = 0
            y_sum = 0
            for i in range(len(data_list)):
                x_, y_ = data_list[i][0], data_list[i][1]  # Unpack the tuple
                x_sum += x_
                y_sum += y_
            x_m = x_sum / len(data_list)  # Calculate mean of x coordinates
            y_m = y_sum / len(data_list)
        else:
            x_sum = 0
            y_sum = 0
            for i in range(len(data_list)):
                x_, y_ = data_list[i][0], data_list[i][1]  # Unpack the tuple
                x_sum += x_
                y_sum += y_
            if len(data_list) != 0:
                x_m = x_sum / len(data_list)  # Calculate mean of x coordinates
                y_m = y_sum / len(data_list)
            else:
                x_m = x_sum / 1  # Calculate mean of x coordinates
                y_m = y_sum / 1
        return int(x_m), int(y_m)

    def trackedCenter(self, model, classes, frame1, frame2):
        left_image_np = np.array(frame1)
        right_image_np = np.array(frame2)

        # # Perform inference
        left_results = model(left_image_np, classes=classes, conf=0.6)
        right_results = model(right_image_np, classes=classes, conf=0.6)

        left_mask = self.make_segmentation_masks(left_results, left_image_np)
        right_mask = self.make_segmentation_masks(right_results, right_image_np)

        left_center = self.compute_centroid_via_pixels_top(left_mask)
        right_center = self.compute_centroid_via_pixels_top(right_mask)

        ###########################################
        # moving average filter
        mean_left = self.mean_window(camera_location="l", data=left_center, length=5)
        mean_right = self.mean_window(camera_location="r", data=right_center, length=5)
        ###########################################

        return mean_left, mean_right

    def trackedCenterShow(self, model, classes, imgleft, imgright):
        # # Perform inference
        left_results = model(imgleft, classes=classes, conf=0.6)
        right_results = model(imgright, classes=classes, conf=0.6)

        left_mask = self.make_segmentation_masks(left_results, imgleft)
        right_mask = self.make_segmentation_masks(right_results, imgright)

        # left_annotated_frame = left_results[0].plot()
        # right_annotated_frame = right_results[0].plot()

        left_center = self.compute_centroid_via_pixels_top(left_mask)
        right_center = self.compute_centroid_via_pixels_top(right_mask)

        ###########################################
        # moving average filter
        mean_left = self.mean_window(camera_location="l", data=left_center, length=5)
        mean_right = self.mean_window(camera_location="r", data=right_center, length=5)
        ###########################################

        self.draw_over_image_inplace(imgleft, mean_left)
        self.draw_over_image_inplace(imgright, mean_right)

        return mean_left, mean_right

    def compute_cup_min_max(self, mask_image):
        xmean, ymean, xmin, xmax, ymin, ymax = 0, 0, 0, 0, 0, 0
        if np.any(mask_image):
            Y, X = np.nonzero(mask_image)
            xmean = int(np.mean(X))
            ymean = int(np.mean(Y))
            xmin = int(np.min(X))
            xmax = int(np.max(X))
            ymin = int(np.min(Y))
            ymax = int(np.max(Y))
        return (xmean, ymean), (xmin, ymean), (xmax, ymean), (xmean, ymin), (xmean, ymax)

    def trackedMinMax(self, model, classes, frame1, frame2):
        left_image_np = np.array(frame1)
        right_image_np = np.array(frame2)

        # # Perform inference
        left_results = model(left_image_np, classes=classes, conf=0.6)
        right_results = model(right_image_np, classes=classes, conf=0.6)

        left_mask = self.make_segmentation_masks(left_results, left_image_np)
        right_mask = self.make_segmentation_masks(right_results, right_image_np)
        l_cent, l_left, l_right, l_top, l_bottom = self.compute_cup_min_max(left_mask)
        r_cent, r_left, r_right, r_top, r_bottom = self.compute_cup_min_max(right_mask)

        return [l_cent, l_left, l_right, l_top, l_bottom], [r_cent, r_left, r_right, r_top, r_bottom]

    def trackedAllPoint(self, model, classes, frame1, frame2):
        leftpoints, rightpoints = self.trackedMinMax(model, classes, frame1, frame2)

        for uv in leftpoints:
            self.draw_over_image(frame1, uv)
        for uv in rightpoints:
            self.draw_over_image(frame2, uv)

        return leftpoints, rightpoints


if __name__ == "__main__":
    from ultralytics import YOLO
    import cv2

    # Define the classes for 'cup' and 'wine glass'
    classes = [40, 41]  # Class IDs in YOLO for 'wine glass' and 'cup'

    # Load the YOLO model
    model = YOLO("/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt")
    cap1 = cv2.VideoCapture(4)
    cap2 = cv2.VideoCapture(10)

    center = Center()

    while cap1.isOpened():
        # Read a frame from the video
        success1, frame1 = cap1.read()
        success2, frame2 = cap2.read()

        if success1 and success2:

            # left_center, right_center = center.trackedCenter(model, classes, frame1, frame2)
            # print(left_center, right_center)
            # left, right = center.trackedCenterShow(model, classes, frame1, frame2)
            leftpoints, rightpoints = center.trackedAllPoint(model, classes, frame1, frame2)

            cv2.imshow("Left Inference", frame1)
            cv2.imshow("Right Inference", frame2)

            # Break the loop if 'esc' is pressed
            if cv2.waitKey(1) & 0xFF == 27:
                break
        else:
            break
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
