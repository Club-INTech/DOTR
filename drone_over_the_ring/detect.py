import torch
import numpy as np


from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import (LOGGER, Profile, check_img_size, cv2, non_max_suppression, scale_boxes)
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import select_device, smart_inference_mode
from yolov5.utils.augmentations import letterbox

def load_model(weights, device='', imgsz=(640, 640)):

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device)
    imgsz = check_img_size(imgsz, s=model.stride)  # check image size

    return model

@smart_inference_mode()
def run(
        model,
        im0,
        imgsz=(640, 640),
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        line_thickness=3  # bounding box thickness (pixels)
):
    stride, names, pt = model.stride, model.names, model.pt
    
    # Image Preprocessing
    im = letterbox(im0, imgsz, stride, auto=True)[0]  # padded resize
    im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    im = np.ascontiguousarray(im)  # contiguous
    
    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else 1, 3, *imgsz))  # warmup
    seen, dt = 0, (Profile(), Profile(), Profile())
    with dt[0]:
        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

    # Inference
    with dt[1]:
        pred = model(im, augment=False, visualize=False)
    # NMS
    with dt[2]:
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    # Second-stage classifier (optional)
    # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1
        im0 = im0.copy()

        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)  # integer class
                label = f'{names[c]} {conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(c, True))
  
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    
    return im0, det
