import torch
import cv2

class YoloDetect():
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.objects = []

    def detect(self, img):
        print("Detecting objects")
        self.objects = []
        result = self.model(img)

        data_frame = result.pandas().xyxy[0]
        indexes = data_frame.index

        print(data_frame)

        for index in indexes:
            # Find the coordinate of top left corner of bounding box
            x1 = int(data_frame['xmin'][index])
            y1 = int(data_frame['ymin'][index])
            # Find the coordinate of right bottom corner of bounding box
            x2 = int(data_frame['xmax'][index])
            y2 = int(data_frame['ymax'][index ])

            # Find label name
            label = data_frame['name'][index ]
            # Find confidance score of the model
            conf = data_frame['confidence'][index]
            text = label + ' ' + str(conf.round(decimals= 2))

            # Add the object to the list
            self.objects.append({
                'description': label,
                'position': (x1, y1, x2, y2),
                'confidence': conf
            })

        return self.objects