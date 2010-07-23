
#include "alvision/alimage.h"
#include "alvision/alvisiondefinitions.h"

#include "manconfig.h"

#include "MmapImageTranscriber.h"
#include "corpusconfig.h"
#include "CameraConstants.h"

#ifdef DEBUG_ALIMAGE
#  define DEBUG_ALIMAGE_LOOP
#endif

using boost::shared_ptr;
using namespace AL;
#define USE_MMAP

int MmapImageTranscriber::xioctl (int fd, int request, void * arg)
{
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

MmapImageTranscriber::MmapImageTranscriber(shared_ptr<Synchro> synchro,
                                           shared_ptr<Sensors> s,
                                           ALPtr<ALBroker> broker)
    : ThreadedImageTranscriber(s,synchro,"MmapImageTranscriber"),
      log(), camera_active(false),
      image(new unsigned char[IMAGE_BYTE_SIZE])
{
    try {
        log = broker->getLoggerProxy();
        // Possible values are
        // lowDebug, debug, lowInfo, info, warning, error, fatal
        log->setVerbosity("error");
    }catch (ALError &e) {
        std::cerr << "Could not create a proxy to ALLogger module" << std::endl;
    }

#ifdef USE_VISION
    if (registerCamera()){
        camera_active = true;
        initializeBuffers();
        initCameraSettings();
    }
    else {
        camera_active = false;
        std::cout << "\tCamera is inactive!" << std::endl;
    }
#endif
}

MmapImageTranscriber::~MmapImageTranscriber() {
    stop();
    //delete[] buffers;
}

int MmapImageTranscriber::start() {
    return Thread::start();
}

void MmapImageTranscriber::start_streaming() {

    // enqueue buffers to be filled
    v4l2_buffer buffer;
    for (unsigned int i = 0; i < REQUIRED_BUFFERS; i++){
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if ( 0 > xioctl(sd, VIDIOC_QBUF, &buffer)){
            log->error("MmapImageTranscriber", "problem queueing buffer");
        }
    }

    memset(&current_frame, 0, sizeof(current_frame));
    current_frame.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    current_frame.memory = V4L2_MEMORY_MMAP;
    //current_frame.index = buffer.index;

    memset(&last_frame, 0, sizeof(last_frame));
    last_frame.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    last_frame.memory = V4L2_MEMORY_MMAP;
    //last_frame.index = buffer.index - 1;

    v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 > xioctl(sd,VIDIOC_STREAMON, &type)){
        log->error("MmapImageTranscriber", "video streaming failed to start");
    }
}

void MmapImageTranscriber::run() {
    Thread::running = true;
    Thread::trigger->on();

    start_streaming();

    long long lastProcessTimeAvg = VISION_FRAME_LENGTH_uS;

    struct timespec interval, remainder;
    while (Thread::running) {
        //start timer
        const long long startTime = micro_time();

        if (camera_active){
            fd_set fds;
            struct timeval tv;
            int r;
            FD_ZERO (&fds);
            FD_SET (sd, &fds);

            /* Timeout. */
            tv.tv_sec = 5;
            tv.tv_usec = 0;
            // if (-1 == xioctl (sd, VIDIOC_QUERYBUF, &current_frame)) {
            //     perror ("VIDIOC_QUERYBUF");
            //     exit (EXIT_FAILURE);
            // }

            // std::cout << " mapped: " << current_frame.flags << std::endl;

            // r = select (sd + 1, &fds, NULL, NULL, &tv);

            // if (-1 == r) {
            //     if (EINTR == errno){
            //         log->error("MmapImageTranscriber", "select");
            //     }
            // }

            // if (0 == r) {
            //     log->error("MmapImageTranscriber","select timeout");
            // }
            waitForImage();
        }

        subscriber->notifyNextVisionImage();

        //stop timer
        const long long processTime = micro_time() - startTime;
        //sleep until next frame

        lastProcessTimeAvg = lastProcessTimeAvg/2 + processTime/2;

        if (processTime > VISION_FRAME_LENGTH_uS) {
            if (processTime > VISION_FRAME_LENGTH_PRINT_THRESH_uS) {
                //#ifdef DEBUG_ALIMAGE_LOOP
                std::cout << "Time spent in MmapImageTranscriber loop longer than"
                          << " frame length: " << processTime <<std::endl;
                //#endif
            }
            //Don't sleep at all
        } else{
            const long int microSleepTime = (VISION_FRAME_LENGTH_uS -
                                             processTime);
            const long int nanoSleepTime =
                (microSleepTime %(1000 * 1000)) * 1000;

            const long int secSleepTime = microSleepTime / (1000*1000);

            // std::cout << "Sleeping for nano: " << nanoSleepTime <<
            //  	" and sec:" << secSleepTime << std::endl;

            interval.tv_sec = secSleepTime;
            interval.tv_nsec = nanoSleepTime;

            //nanosleep(&interval, &remainder);
        }
    }
    Thread::trigger->off();
}

void MmapImageTranscriber::stop() {
    std::cout << "Stopping MmapImageTranscriber" << std::endl;
    running = false;
#ifdef USE_VISION
    if(camera_active){
        stop_streaming();
        close(sd);
        sd = -1;
    }
    for (unsigned int i = 0; i < REQUIRED_BUFFERS; i++){
        munmap (buffers[i].start, buffers[i].length);
    }

#endif

    Thread::stop();
}

void MmapImageTranscriber::stop_streaming() {
    v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 > xioctl(sd,VIDIOC_STREAMOFF, &type)){
        log->error("MmapImageTranscriber", "video streaming failed to stop");
    }
}

bool MmapImageTranscriber::registerCamera() {
    sd = open ("/dev/video0", O_RDWR | O_NONBLOCK, 0);
    if (0 > sd){
        log->error("MmapImageTranscriber",
                   "Could not attach to video device");
        return false;
    }

    return true;
}

void MmapImageTranscriber::initializeBuffers(){

    // setup memory map
    v4l2_requestbuffers reqbuf;
    memset (&reqbuf, 0, sizeof (reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = REQUIRED_BUFFERS;

    if (-1 == xioctl (sd, VIDIOC_REQBUFS, &reqbuf)) {
        if (errno == EINVAL)
            printf ("Video capturing or mmap-streaming is not supported\n");
        else
            perror ("VIDIOC_REQBUFS");

        exit (EXIT_FAILURE);
    }
    std::cout << "count: " << reqbuf.count << std::endl;

    // zero initialize buffers array
    memset (&buffers, 0, sizeof (buffers));

    for (unsigned int i = 0; i < reqbuf.count; i++) {
        v4l2_buffer buffer;

        memset (&buffer, 0, sizeof (buffer));
        buffer.type = reqbuf.type;
	buffer.memory = reqbuf.memory;
        buffer.index = i;

        if (-1 == xioctl (sd, VIDIOC_QUERYBUF, &buffer)) {
            perror ("VIDIOC_QUERYBUF");
            exit (EXIT_FAILURE);
        }

        buffers[i].length = buffer.length; /* remember for munmap() */

        buffers[i].start = mmap (NULL, buffer.length,
                                 PROT_READ | PROT_WRITE, /* recommended */
                                 MAP_SHARED,             /* recommended */
                                 sd, buffer.m.offset);

        if (MAP_FAILED == buffers[i].start) {
            /* If you do not exit here you should unmap() and free()
               the buffers mapped so far. */
            perror ("mmap");
            exit (EXIT_FAILURE);
        }
    }
}

void MmapImageTranscriber::initCameraSettings(){
    // // set priority of this to highest
    // v4l2_priority p;
    // memset(&p, 0, sizeof(p));
    // if (0 > xioctl(sd, VIDIOC_G_PRIORITY, &p)){
    //     log->error("MmapImageTranscriber", "getting priority failed ");
    // }
    // if (p != V4L2_PRIORITY_RECORD){
    //     // throws error is it's already set to highest
    //     if (0 > xioctl(sd, VIDIOC_S_PRIORITY, &p)){
    //         log->error("MmapImageTranscriber", "setting priority failed ");
    //     }
    // }

    // set video type (VGA, QVGA)
    v4l2_std_id std_id = (v4l2_std_id)esid0;
    if (-1 == xioctl (sd, VIDIOC_S_STD, &std_id)) {
        perror ("VIDIOC_S_STD");
    }

    // set video size and pixel format
    v4l2_format fmt0;
    memset( &fmt0, 0, sizeof( fmt0 ) );
    fmt0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt0.fmt.pix.field = V4L2_FIELD_ANY; //INTERLACED?
    fmt0.fmt.pix.width = NAO_IMAGE_WIDTH;
    fmt0.fmt.pix.height = NAO_IMAGE_HEIGHT;
    fmt0.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (0 != xioctl( sd, VIDIOC_S_FMT, &fmt0 )){
        log->error("MmapImageTranscriber", "failed to set video format");
    }

    v4l2_cropcap cropcap;
    memset(&cropcap, 0, sizeof(cropcap));
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl (sd, VIDIOC_CROPCAP, &cropcap)) {
        v4l2_crop crop;
        memset(&crop, 0, sizeof(crop));
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl (sd, VIDIOC_S_CROP, &crop)) {
            log->error("MmapImageTranscriber", "error setting cropping");
        }
    }
    else {
        log->error("MmapImageTranscriber", "error getting crop info");
    }


    v4l2_streamparm param;
    memset (&param, 0, sizeof(param));
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // get current streaming param
    if (0 > xioctl(sd, VIDIOC_G_PARM, &param)){
        log->error("MmapImageTranscriber", "getting streaming param failed");
    }

    // reset to what we want
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    param.parm.capture.timeperframe.numerator = 1.;
    param.parm.capture.timeperframe.denominator = VISION_FPS;
    param.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

    // set driver to only capture at 30 fps, saving overhead
    if (0 > xioctl(sd, VIDIOC_S_PARM, &param)){
        log->error("MmapImageTranscriber", "setting streaming param failed");
    }
}

void MmapImageTranscriber::waitForImage (){
    // backup last_frame here, so we can enqueue it after we're done w/it
    last_frame = current_frame;
    memset(&current_frame, 0, sizeof(current_frame));
    current_frame.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    current_frame.memory = V4L2_MEMORY_MMAP;

    // this function blocks until frame is available
    if (0 > xioctl(sd, VIDIOC_DQBUF, &current_frame)){
        log->error("MmapImageTranscriber", "could not get frame");
    }

    // Update Sensors image pointer
    sensors->lockImage();
    // image is an unsigned char *
    sensors->setImage(static_cast<unsigned char*>
                      (buffers[current_frame.index].start));
    sensors->releaseImage();

    // enqueue old buffer for reuse
    if (0 > xioctl(sd, VIDIOC_QBUF, &last_frame)){
        perror("VIDIOC_QBUF");
    }
}


void MmapImageTranscriber::releaseImage(){
}
