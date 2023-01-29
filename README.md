# DOTR


# Create your database

Here is the section you can follow to create your database, it is the way we have done it but it isn't the only way.


## Take Images

We took the images with the drone with the scripts **dataset_creator.py** and by pressing the key "i", you just had to change the folder where you want it to save images. Note : the number max of images are 999 999, for the entire dataset creation. If you want to extend it you must change the **index()** methode.
As background images have been added at the end, the current function have to be updated for the images without object when creating .txt, thus you'll want to put the background images at the end of your pool.

## Yolo Format

The label format is a .txt file where each lines match one object.
The format of a line is : **class x_center y_center width height** , where coordinates must be normalized by the dimension of the image and calss numbers are zero-indexed.

Example : 0 0.48225308641975306 0.35133744855967075 0.21682098765432092 0.34053497942386834 

## Annotation of images

We search the easiest way to annote images with bounding boxes. We choose to use LabelMe from MIT. There is a repo github [here](https://github.com/wkentaro/labelme) to install it with pip.
This annotate tool output a JSON format for the labeling, so we have a function call **createLabel()** in **utils_dataset.py** to allow you do to the conversion to YOLO format.
As said earlier we need to adapt the function to the abcense of object during the annotation, so for now you need to add empty .txt at the end to match bg images.

## Augmented data

We double our database with an offline data augmentation, which is a simple horizontal flip, to do so use the funtion **create_augmented_data()**.

## Split of the dataset

We want this arborecence :

For the training we need to split our pool in two (or three) part, to do so you can use the **split()** function that split 90/10 for training and validation.


# Notes

PyAv lib (bindings of ffmpeg) uses its own instance of ffmpeg lib which conflicts with one used by OpenCV. The initialization of ffmpeg is done on the import of **av**. In order to prevent PyAv to use its own ffmpeg we must install it without binaries and bring OS native ffmpeg as alternative. Run the following commands to do so:

```bash
    $ sudo apt-get install libxvidcore-dev libx264-dev
    $ sudo apt-get install libavcodec-dev libavformat-dev \
    libswscale-dev libv4l-dev
    $ sudo apt-get install libavdevice-dev
    $ sudo apt-get install libavfilter-dev
    $ pip3 install av --no-binary av
```