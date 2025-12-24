import cv2
import numpy

#入力で受け取った画像を指定したピクセルで平均を取りT/Fの二次元配列にして返す.
def img_to_grid(image) -> list[list[bool]]:

    #必要に応じて調整.
    cell_size_px = 10
    threshold_ratio=1.0

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray_image.shape

    # test
    # print('h: ', h)
    # print('w: ', w)

    # rough_w = w // cell_size_px
    # rough_h = h // cell_size_px

    # test
    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # rough_image = cv2.resize(gray_image, (rough_w, rough_h), interpolation=cv2.INTER_AREA)
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    #test
    # cv2.imshow('image', rough_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # mean_brightness = numpy.mean(rough_image)
    # print(mean_brightness)

    # prosessed_image = cv2.adaptiveThreshold(rough_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 0)
    prosessed_image = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 101, 0)

    cv2.imshow('image', prosessed_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    result = [[0] * w for _ in range(h)]

    for i in range(h):
        for j in range(w):
            if prosessed_image[i][j] > 0 :
                result[i][j] = True
            else:
                result[i][j] = False

    return result

#テスト用
if __name__ == "__main__":
    print(cv2.__version__)
    image = cv2.imread('data_for_test/image.jpg')
    lis = img_to_grid(image)

    for i in range(len(lis)):
        print(lis[i])