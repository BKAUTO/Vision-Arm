import cv2

img = cv2.imread('test1.bmp', cv2.IMREAD_GRAYSCALE)
print(img.dtype)
print(img.size)

def onmouse(event, x, y, flags, param):
  if event == cv2.EVENT_MOUSEMOVE:
    print(y, x, img[y, x])


cv2.namedWindow("img")
cv2.setMouseCallback("img", onmouse)
while True:
    cv2.imshow("img", img)
    if cv2.waitKey() == ord('q'):
        break
cv2.destroyAllWindows()