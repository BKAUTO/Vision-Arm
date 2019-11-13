import cv2
from telnet_connect import TelnetClient
telnet_client = TelnetClient()
if telnet_client.login_host('192.168.2.99', 'admin', ''):
	telnet_client.execute_command('SE8')
	telnet_client.save_img('grasp_test')
	telnet_client.logout_host

img = cv2.imread('grasp_test.bmp', cv2.IMREAD_GRAYSCALE)
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