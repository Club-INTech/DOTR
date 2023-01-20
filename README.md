# DOTR

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