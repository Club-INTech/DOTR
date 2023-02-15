import cv2
import utils_dataset

# Scripts to validate the dataset, going trough all the images and checking the bounding boxes

# Size of the image - constant in the current project
IMAGE_WIDTH = 960
IMAGE_HEIGHT = 720

# Path of the directory containing the dataset
ROOT_path = r'D:\DATASET_FINAL_DRONE'

# There should have both folder named "images" and "label" in this directory (ROOT_path)

# Add the bounding box to the image from the .txt file
def print_bboxesYOLO(img,bboxes):
    if len(bboxes) == 0:
        return img
    for k in range(len(bboxes)):
        x1 = int( (bboxes[k][0]-bboxes[k][2]/2)*IMAGE_WIDTH )
        y1 = int( (bboxes[k][1]-bboxes[k][3]/2)*IMAGE_HEIGHT )
        x2 = int( (bboxes[k][0]+bboxes[k][2]/2)*IMAGE_WIDTH )
        y2 = int( (bboxes[k][1]+bboxes[k][3]/2)*IMAGE_HEIGHT )
        cv2.rectangle(img, (x1, y1), (x2,y2), (0, 0, 255), 2)
        cv2.putText(img, str(bboxes[k][4]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return img

img = cv2.imread(ROOT_path+r'\images\image000000.jpg')
bboxes = utils_dataset.ReadTxt(ROOT_path+r'\label\image000000.txt')
img_to_show = print_bboxesYOLO(img,bboxes)
num_image = 0

while True:
    name_num = str('{:06d}'.format(num_image))
    cv2.imshow("Main Screen",img_to_show)
    
    key = cv2.waitKey(1) & 0xFF
    # Next Image with key D, Previous with key Q, Note current image with key Z, Exit with key A
    if key == ord('a'):
        break
    if key == ord('d'):
        num_image += 1
        img = cv2.imread(ROOT_path+r'\images\image'+name_num+'.jpg')
        bboxes = utils_dataset.ReadTxt(ROOT_path+r'\label\image'+name_num+'.txt')
        img_to_show = print_bboxesYOLO(img,bboxes)
    elif key == ord('q'):
        num_image -= 1
        img = cv2.imread(ROOT_path+r'\images\image'+name_num+'.jpg')
        bboxes = utils_dataset.ReadTxt(ROOT_path+r'\label\image'+name_num+'.txt')
        img_to_show = print_bboxesYOLO(img,bboxes)
    if key == ord('z'):
        print('num_image : ',num_image)