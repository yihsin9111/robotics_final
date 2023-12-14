import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model

class Gesture:
    def __init__(self):
        # initialize mediapipe & load model
        print('Initializing gesture detection...')
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils
        self.model = load_model('mp_hand_gesture')

        # load class names
        f = open('gesture.names', 'r')
        self.classNames = f.read().split('\n')
        f.close()
        print(self.classNames)

        # gesture prediction & position
        self.pred = 0 # gesture prediction
        self.x = 0
        self.y = 0
        self.z = 0
        

    def get_gesture(self, img):
        x, y, c = img.shape

        # Flip the frame vertically
        img = cv2.flip(img, 1)
        imgrgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Get hand landmark prediction
        result = self.hands.process(imgrgb)
        #print(result.multi_hand_landmarks)

        # post process the result
        if result.multi_hand_landmarks:
            landmarks = []
            for handslms in result.multi_hand_landmarks:
                for lm in handslms.landmark:
                    # print(id, lm)
                    lmx = int(lm.x * x)
                    lmy = int(lm.y * y)

                    landmarks.append([lmx, lmy])

                # Drawing landmarks on frames
                self.mpDraw.draw_landmarks(img, handslms, self.mpHands.HAND_CONNECTIONS)

                # Predict gesture
                prediction = self.model.predict([landmarks])
                # print(prediction)
                classID = np.argmax(prediction)
                self.pred = self.classNames[classID]

        return img



if __name__ == '__main__':

    gesture = Gesture()
    cap = cv2.VideoCapture(1)

    while True:

        _, img = cap.read()
        cv2.imshow('img', img)

        if cv2.waitKey(1) == ord('q'):
            g_img = gesture.get_gesture(img)
            cv2.imshow('result', img)
            print(f"prediction: {gesture.pred}")
    