import torch
import numpy as np
from gate_descriptor import GateDescriptor, GateType

from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import (LOGGER, Profile, check_img_size, cv2, non_max_suppression, scale_boxes)
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import select_device, smart_inference_mode
from yolov5.utils.augmentations import letterbox

from camera import CAMERA_HEIGHT, CAMERA_WIDTH

class GateDetector():
    
    def __init__(self, weights, device='', imgsz=(640, 640)):
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device)
        self.imgsize = check_img_size(imgsz, s=self.model.stride)  # check image size

        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.line_thickness=3  # bounding box thickness (pixels)

    @smart_inference_mode()
    def run(self, im0):
        stride, names, pt = self.model.stride, self.model.names, self.model.pt
        
        # Image Preprocessing
        
        assert im0 is not None, f'Image Not Found'
        
        im = letterbox(im0, self.imgsize, stride, auto=True)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous
        
        # Run inference
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *self.imgsize))  # warmup
        seen, dt = 0, (Profile(), Profile(), Profile())
        with dt[0]:
            im = torch.from_numpy(im).to(self.model.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            pred = self.model(im, augment=False, visualize=False)
        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            im0 = im0.copy()

            annotator = Annotator(im0, line_width=self.line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
    
        t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
        LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *self.imgsize)}' % t)
        
        # To DO : clean the function to annotate here and create an external methode / we can currently put im0 as the image annoted for the output at this stage
        
        
        # Convertion of det to a gate_descriptor
        
        det = det.cpu()

        if(len(det) > 0):
            heights = []
            for i in range(len(det)):
                heights.append(det[i][3]-det[i][1])
            
            gate_nb = heights.index(max(heights))    
                
            if(det[gate_nb][5] == 0):
                type_ = GateType.CIRCLE_GATE
            elif(det[gate_nb][5] == 1):
                type_ = GateType.SQUARE_GATE
            elif(det[gate_nb][5] == 2):
                type_ = GateType.HEX_GATE
                
            gd = GateDescriptor(pixel_width=det[gate_nb][2]-det[gate_nb][0],
                                    pixel_height=det[gate_nb][3]-det[gate_nb][1],
                                    score=det[gate_nb][4],
                                    type_= type_)
            gd.set_xyz_from_image(det[gate_nb][0], det[gate_nb][1], det[gate_nb][2], det[gate_nb][3])
            
        else:
            gd = GateDescriptor()
        
        return im0, gd