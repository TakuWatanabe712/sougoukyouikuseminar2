import cv2
import numpy

#入力で受け取った画像を指定したピクセルで平均を取りT/Fの二次元配列にして返す.
def img_to_grid(image) -> list[bool][bool]:

    #必要に応じて調整.
    cell_size_px = 10, threshold_ratio=1.0

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray_image.shape

    