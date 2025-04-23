# importing cv2
import cv2

#For Video File
#capture=cv2.VideoCapture("sample.webm")

#For webcam
capture=cv2.VideoCapture(0)

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3264)

capture.set(cv2.CAP_PROP_FRAME_HEIGHT,2448)

# showing values of the properties
print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print("CAP_PROP_FPS : '{}'".format(capture.get(cv2.CAP_PROP_FPS)))
print("CAP_PROP_POS_MSEC : '{}'".format(capture.get(cv2.CAP_PROP_POS_MSEC)))
print("CAP_PROP_FRAME_COUNT : '{}'".format(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
print("CAP_PROP_BRIGHTNESS : '{}'".format(capture.get(cv2.CAP_PROP_BRIGHTNESS)))
print("CAP_PROP_CONTRAST : '{}'".format(capture.get(cv2.CAP_PROP_CONTRAST)))
print("CAP_PROP_SATURATION : '{}'".format(capture.get(cv2.CAP_PROP_SATURATION)))
print("CAP_PROP_HUE : '{}'".format(capture.get(cv2.CAP_PROP_HUE)))
print("CAP_PROP_GAIN : '{}'".format(capture.get(cv2.CAP_PROP_GAIN)))
print("CAP_PROP_CONVERT_RGB : '{}'".format(capture.get(cv2.CAP_PROP_CONVERT_RGB)))
print("CAP_PROP_FOCUS : '{}'".format(capture.get(cv2.CAP_PROP_FOCUS)))
print("CAP_PROP_AUTOFOCUS : '{}'".format(capture.get(cv2.CAP_PROP_AUTOFOCUS)))

# release window
capture.release()
cv2.destroyAllWindows()
