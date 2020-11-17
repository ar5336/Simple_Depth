import numpy as np
import cv2
import time


def simple_depth(left_path, right_path):
    left_image = cv2.imread(left_path)
    right_image = cv2.imread(right_path)

    og_height, og_width, _ = left_image.shape

    R_SHIFT_UP = 32

    v_shift_transform = np.float32([[1, 0, 0], [0, 1, -R_SHIFT_UP]])
    right_image = cv2.warpAffine(right_image, v_shift_transform, (og_width, og_height))

    height = int(og_height/2)
    width = int(og_width/2)

    print(str(width) +", "+str(height))

    left_image = cv2.resize(left_image, (0,0), fx=.5, fy=.5)
    right_image = cv2.resize(right_image, (width, height))

    map_stack = []

    # SHOUDL I HAVE A FOCUS DISTANCE??

    START = 30
    END = 70
    STEP_SIZE = 2
    right_im_left_shift = np.float32([[1, 0, START], [0, 1, 0]])
    right_image = cv2.warpAffine(right_image, right_im_left_shift, (width, height))
    for i in range(START, END, STEP_SIZE):
        diff_map = ~cv2.absdiff(left_image, right_image)

        map_stack.append(diff_map)

        right_im_left_shift = np.float32([[1,0,STEP_SIZE],[0,1,0]])
        right_image = cv2.warpAffine(right_image, right_im_left_shift, (width, height))

    for i in range(1, len(map_stack)):
        prev_frame = map_stack[i-1]
        right_shift = np.float32([[1, 0, 2], [0, 1, 0]])
        prev_frame = cv2.warpAffine(prev_frame, right_shift, (width, height))
        map_stack[i-1] = cv2.absdiff( map_stack[i], prev_frame)
    map_stack = map_stack[:-1]

    for i in range(1, len(map_stack)):
        prev_frame = map_stack[i-1]
        map_stack[i - 1] = cv2.subtract(map_stack[i], prev_frame)
    map_stack = map_stack[:-1]

    for i in range(len(map_stack)):
        map = map_stack[i]
        map_r = map[:,:,2]
        map_g = map[:,:,1]
        map_b = map[:,:,0]
        map_white = np.minimum(np.minimum(map_b, map_g), map_r)

        map_stack[i] = np.multiply(map_white, 30)

    # Running average
    new_map_stack = []
    filter_size = 1

    prev_frame_stack = []
    prev_frame_total = np.zeros((height, width))
    for i in range(len(map_stack)):
        if len(prev_frame_stack) < filter_size:
            prev_frame_total = np.add(prev_frame_total, map_stack[i])
            prev_frame_stack.append(prev_frame_total)
            new_map_stack.append(prev_frame_total)
        else:
            prev_frame_total = np.subtract(prev_frame_total, prev_frame_stack[0])
            del prev_frame_stack[0]
    map_stack = new_map_stack

    for j in range(len(map_stack)):
        map_stack[j] = cv2.blur(np.float32(map_stack[j]), (3,3))
        map_stack[j] = cv2.medianBlur(map_stack[j], 5)

    # for i in range(1, len(map_stack)):
    #     prev_frame = map_stack[i-1]
    #     map_stack[i - 1] = cv2.subtract(map_stack[i], prev_frame)
    # map_stack = map_stack[:-1]

    print("DONE")
    count = 0
    keep_going = True
    while True:
        if keep_going:
            cv2.imshow("maps", map_stack[count % len(map_stack)])
            time.sleep(.05)
        count += 1
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord(' '):
            keep_going = not keep_going
        if key == ord('n'):
            continue


if __name__ == "__main__":
    IM_NUM = 2
    simple_depth("im_"+str(IM_NUM)+"/left_im_"+str(IM_NUM)+".jpg", "im_"+str(IM_NUM)+"/right_im_"+str(IM_NUM)+".jpg")