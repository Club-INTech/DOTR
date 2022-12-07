import cv2
import os
import albumentations as A
import json

# Path of the images taken by the drone
PATH_TO_SAVE = r'D:\DATASET_Drone-Test'

# ====================================================================== Images managing on directory =================================================================================

def remove_image(index):
    os.remove(PATH_TO_SAVE + r'\image'+str('{:06d}'.format(index))+r'.jpg')
    print(len(os.listdir(PATH_TO_SAVE)))
    for i in range(index+1, len(os.listdir(PATH_TO_SAVE))+1):
        os.rename(PATH_TO_SAVE + r'\image'+str('{:06d}'.format(i))+r'.jpg', PATH_TO_SAVE + r'\image'+str('{:06d}'.format(i-1))+r'.jpg')

def index(path):
    num = len(os.listdir(path))
    output = str('{:06d}'.format(num))
    return output

# ====================================================================== Bounding boxes class for conversion of format ================================================================

# This is the class of bounding box from the software Labelme
class bbox_labelme: 
    def __init__(self, label, id, point_1, point_2, image_name):
        self.label = label
        self.id = id
        self.point_1 = point_1
        self.point_2 = point_2
        self.image_name = image_name
        
    def __str__(self):
        return f'From Image : {self.image_name}, Label: {self.label}, Id: {self.id}, Point 1: {self.point_1}, Point 2: {self.point_2}'
    
    def toYolo(self,image_size):    # X = Width, Y = Height
        
        # Points before checking the format, and  the verification of their order
        _x1 = self.point_1[0]
        _y1 = self.point_1[1]
        _x2 = self.point_2[0]
        _y2 = self.point_2[1]
        
        if _y1 < _y2 and _x1 > _x2:
            print("Debug : Inverted corner - anti diagonal of the square & up to down" + f' on image : {self.image_name}')
            x1 = _x2
            y1 = _y1
            x2 = _x1
            y2 = _y2
        if _y1 > _y2 and _x1 < _x2:
            print("Debug : Inverted corner - anti diagonal of the square & down to up" + f' on image : {self.image_name}')
            x1 = _x1
            y1 = _y2
            x2 = _x2
            y2 = _y1
        if _y1 > _y2 and _x1 > _x2:
            print("Debug : Inverted corner - diagonal of the square & down to up" + f' on image : {self.image_name}')
            x1 = _x2
            y1 = _y2
            x2 = _x1
            y2 = _y1
        if _y1 < _y2 and _x1 < _x2:
            x1 = _x1
            y1 = _y1
            x2 = _x2
            y2 = _y2
        
        w = x2 - x1
        h = y2 - y1
        
        if w<0 or h<0:
            print("Error : Negative width or height" + f' on image : {self.image_name}')
            return None
        
        x_center = x1 + w/2
        y_center = y1 + h/2
        x_center = x_center/image_size[1]
        y_center = y_center/image_size[0]
        w = w/image_size[1]
        h = h/image_size[0]
        return bbox_Yolo(self.label, self.id, x_center, y_center, w, h, self.image_name)
        
class bbox_Yolo:
    def __init__(self, label, id, center_x, center_y, width, height, image_name):
        self.label = label
        self.id = id
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.image_name = image_name

    def __str__(self):
        return f'From Image : {self.image_name}, Label: {self.label}, Id: {self.id}, Center_x: {self.center_x}, Center_y: {self.center_y}, Width: {self.width}, Height: {self.height}'

    def toListforYOlo(self):
        return [self.id, self.center_x, self.center_y, self.width, self.height]
    
    def toTXT(self):
        data = self.toListforYOlo()
        string = ""
        for i in range(len(data)-1):
            string += str(data[i]) + " "
        return string + str(data[-1])


# ==================================================================== Functions to read a JSON file and write a TXT as label for training ============================================

# Extraction of the bounding boxes from ONE image
def JSONextraction(file): # X = height, Y = width
    with open(file, 'r') as f:
        data = json.load(f)
        object = data['shapes']
        bboxes = []
        image_param = [data['imageHeight'], data['imageWidth']]
        for i in range(len(object)):
            bbox = bbox_labelme(object[i]['label'], object[i]['group_id'], object[i]['points'][0], object[i]['points'][1], data['imagePath'][-15:])
            bboxes.append(bbox)
        return bboxes, image_param

# Creation of ONE file .txt for Yolo format for ONE image
def TXTcreation(bboxes_YOLO,path):
    image_name = bboxes_YOLO[0].image_name[:-4] + '.txt'
    file = open(path+r"\\"+image_name, "w")
    print(path+r"\\"+image_name)
    for k in range(len(bboxes_YOLO)-1):
        file.write(bboxes_YOLO[k].toTXT()+'\n')
    file.write(bboxes_YOLO[-1].toTXT())
    file.close()

def ReadTxt(path):
    file = open(path, 'r')
    bboxes = []
    for k in file.readlines():
        data = k.split(' ')
        bboxes.append([float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[0])])
    return bboxes

# ==================================================================== Creation a the whole directory of label for training ===========================================================

def createLabel(path_to_json, path_to_save):
    files = os.listdir(path_to_json)
    for i in range(len(files)):
        bboxes, image_param = JSONextraction(path_to_json + r'\\'+ files[i])
        bboxes_YOLO = []
        for j in range(len(bboxes)):
            bboxes_YOLO.append(bboxes[j].toYolo(image_param))
        TXTcreation(bboxes_YOLO, path_to_save)

# ==================================================================== Data Augmentation ==============================================================================================

def data_augmentation_horizontal(img,label):
    # Horizontal flip
    transform = A.Compose([
        A.HorizontalFlip(p=1),], bbox_params=A.BboxParams(format='yolo'))
    print(label)
    transformed = transform(image=img, bboxes=label)
    transformed_image = transformed["image"]
    transformed_bboxes = transformed["bboxes"]
    return transformed_image, transformed_bboxes

def save_augmented_data(transformed_img,transformed_bboxes,path): # Path = path of the dataset
    cv2.imwrite(path + r'\images\image'+str(index(path+r'\images'))+r'.jpg', transformed_img)  
    file = open(path + r'\label\image'+str(index(path+'\label'))+r'.txt', "w")
    if len(transformed_bboxes) == 0:
        file.close()
        return True
    for k in range(len(transformed_bboxes)-1):
        file.write(str(transformed_bboxes[k][4]) + " " + str(transformed_bboxes[k][0]) + " " + str(transformed_bboxes[k][1]) + " " + str(transformed_bboxes[k][2]) + " " + str(transformed_bboxes[k][3]) + '\n')
    file.write(str(transformed_bboxes[-1][4]) + " " + str(transformed_bboxes[-1][0]) + " " + str(transformed_bboxes[-1][1]) + " " + str(transformed_bboxes[-1][2]) + " " + str(transformed_bboxes[-1][3]))
    file.close()

# ==================================================================== Add of the augmented data =======================================================================================

def create_augmented_data(path_to_images,path_to_labels):
    files = os.listdir(path_to_images)
    NB_imgs = len(files)
    for i in range(NB_imgs):
        print(i)
        img = cv2.imread(path_to_images + r'\\'+ files[i])
        bboxes = ReadTxt(path_to_labels + r'\\'+ files[i][:-4] + '.txt')
        transformed_img, transformed_bboxes = data_augmentation_horizontal(img,bboxes)
        save_augmented_data(transformed_img,transformed_bboxes,r'D:\DATASET_FINAL_DRONE\test_augmented_data')

# ==================================================================== Creation of the split ===========================================================================================

def split(path_to_dataset): # Split data from training directories to validation directories and test directories
    files = os.listdir(path_to_dataset + r'\images\train')
    NB_imgs = len(files)
    count = 0
    for i in range(NB_imgs):
        if i % 10 == 0:
            count += 1
            os.rename(path_to_dataset + r'\images\train\\'+ files[i], path_to_dataset + r'\images\val\\'+ files[i])
            os.rename(path_to_dataset + r'\labels\train\\'+ files[i][:-4] + '.txt', path_to_dataset + r'\labels\val\\'+ files[i][:-4] + '.txt')
    print("Moving " + str(count) + " images from train to val out of " + str(NB_imgs) + " images")

# ==================================================================== End =============================================================================================================

INDEX_end_first_images_batch = 312 # It's the index of the last image taken with the drone