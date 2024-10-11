#!/usr/bin/env python3

from v4l2 import *
import fcntl
import mmap
import select
import time
import struct
import os
import cv2
import numpy as np
import argparse


def main():
    print("V4L2 PLAYER")
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", "-d", dest="device", default='/dev/video2')
    parser.add_argument("--width", "-x", dest="width", default=1920, type=int)
    parser.add_argument("--height", "-y", dest="height", default=1080, type=int)
    parser.add_argument("--bpp", "-b", dest="bpp", default=4, type=int)
    
    parser.add_argument("--position", "-p", dest="position", default="0,0")
    parser.add_argument("--fullscreen", "-f", dest="fullscreen", default=False, action="store_true")
    parser.add_argument("--capture", "-c", dest="capture", default=False, action="store_true")
    
    args = parser.parse_args()
    width = args.width
    height = args.height
    bpp = args.bpp
    
    vd = os.open(args.device, os.O_RDWR, 0)

    print(">> get device capabilities")
    cp = v4l2_capability()
    fcntl.ioctl(vd, VIDIOC_QUERYCAP, cp)

    print("Driver:", "".join((chr(c) for c in cp.driver)))
    print("Name:", "".join((chr(c) for c in cp.card)))
    print("Is a video capture device?", bool(cp.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    print("Supports read() call?", bool(cp.capabilities &  V4L2_CAP_READWRITE))
    print("Supports streaming?", bool(cp.capabilities & V4L2_CAP_STREAMING))

    print(">> device setup")
    fmt = v4l2_format()
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    fcntl.ioctl(vd, VIDIOC_G_FMT, fmt)  # get current settings
    print("width:", fmt.fmt.pix.width, "height", fmt.fmt.pix.height)
    print("pxfmt:", "V4L2_PIX_FMT_YUYV" if fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV else fmt.fmt.pix.pixelformat)
    print("bytesperline:", fmt.fmt.pix.bytesperline)
    print("sizeimage:", fmt.fmt.pix.sizeimage)
    fmt.fmt.pix.height = height
    fmt.fmt.pix.width = width
    fmt.fmt.pix.sizeimage = width * height * bpp
    fmt.fmt.pix.bytesperline = width * bpp
    fcntl.ioctl(vd, VIDIOC_S_FMT, fmt)  # set whatever default settings we got before

    fcntl.ioctl(vd, VIDIOC_G_FMT, fmt)  # get current settings
    print("new width:", fmt.fmt.pix.width, "new height", fmt.fmt.pix.height)
    print("new sizeimage:", fmt.fmt.pix.sizeimage)
    print("new bytesperline:", fmt.fmt.pix.bytesperline)

    print(">> init mmap capture")
    req = v4l2_requestbuffers()
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    req.memory = V4L2_MEMORY_MMAP
    req.count = 1  # nr of buffer frames
    fcntl.ioctl(vd, VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers 
    print("req.count", req.count)

    buffers = []

    print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
    for ind in range(req.count):
        # setup a buffer
        buf = v4l2_buffer()
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = V4L2_MEMORY_MMAP
        buf.index = ind
        fcntl.ioctl(vd, VIDIOC_QUERYBUF, buf)

        buf.buffer =  mmap.mmap(vd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
        print("no {} buf {}",ind,repr(buf))
        buffers.append(buf)

        # queue the buffer for capture
        fcntl.ioctl(vd, VIDIOC_QBUF, buf)

    print(">> Start streaming")
    buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
    fcntl.ioctl(vd, VIDIOC_STREAMON, struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE))#buf_type)

    print(">> Capture image")
    t0 = time.time()
    max_t = 1
    ready_to_read, ready_to_write, in_error = ([], [], [])
    print(">>> select")
    while len(ready_to_read) == 0 and time.time() - t0 < max_t:
        ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)

    print(">>> download buffers")
    
    if args.fullscreen:
        cv2.namedWindow('image', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('image',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    else:
        cv2.namedWindow('image')
    
    pos = args.position.split(",")
    xpos = int(pos[0])
    ypos = int(pos[1])
    cv2.moveWindow('image', xpos, ypos)

    frame_count = 0
    # for _ in range(1000):
    try:
        while(1):
            buf = buffers[frame_count % req.count]
            fcntl.ioctl(vd, VIDIOC_DQBUF, buf)  # get image from the driver queue
            mm = buffers[buf.index].buffer
            frame = np.frombuffer(mm, dtype=np.uint8, count=width*height*bpp)
            frame = np.reshape(frame, (height,width,bpp))

            if bpp == 1:
                # Assuming Bayer format
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
            elif bpp == 2:
                # Assuming Bayer format with 16-bit depth
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
            elif bpp == 3:
                # Assuming BGR format
                frame_bgr = frame
            elif bpp == 4:
                # Assuming BGRA format
                frame[:,:,3] = 255  # Set alpha channel to fully opaque
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            cv2.imshow('image', frame_bgr)
            key = cv2.waitKey(1)
            fcntl.ioctl(vd, VIDIOC_QBUF, buf)  # requeue the buffer

            if(frame_count % 100 == 0):
                print("Frame count:", frame_count)
                if args.capture:
                    filename = "capture_%sx%s.png" % (width, height)
                    cv2.imwrite(filename, frame)
                    print("Capture frame to %s" % filename)

            frame_count += 1
    except KeyboardInterrupt:
        pass

    print(">> Stop streaming")
    fcntl.ioctl(vd, VIDIOC_STREAMOFF, buf_type)
    os.close(vd)
    # vd.close()
    
    
if __name__ == "__main__":
    main()
