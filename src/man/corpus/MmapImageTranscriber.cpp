
#include "manconfig.h"

#include "MmapImageTranscriber.h"
#include "corpusconfig.h"
#include "CameraConstants.h"

#ifdef DEBUG_ALIMAGE
#  define DEBUG_ALIMAGE_LOOP
#endif

using boost::shared_ptr;
using namespace AL;

#define NUM_CONTROLS 8
__u32 controlIds[NUM_CONTROLS] =
    {V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST, V4L2_CID_RED_BALANCE,
     V4L2_CID_BLUE_BALANCE, V4L2_CID_GAIN, V4L2_CID_EXPOSURE,
     V4L2_CID_HFLIP, V4L2_CID_VFLIP};
__s32 controlValues_dark[2][NUM_CONTROLS] =
    {{ 100, 75, 128, 128, 128, 5, false, false},
     { 100, 75, 128, 128, 128, 5, false, false}};
__s32 (*controlValues)[NUM_CONTROLS] = controlValues_dark;

#define CLEAR(x) memset (&(x), 0, sizeof (x))
/**
 * These adresses, commands and flags are given to us by aldeberan. They enable
 * talking to and controlling the camera controller
 * I2C_DEVICE       - the i2c bus that the camera controller resides on
 * I2C_SLAVE        - the flag to indicate slave mode on the i2c bus. allows
 *                    writing to the camera controller
 * DSPIC_I2C_ADDR   - the address of the camera controller on the i2c bus
 * DSPIC_SWITCH_REG - register of the active camera. can be read to determine
 *                    camera or set to change camera.
 *                    currently 0x01 is top camera and 0x02 is bottom.
 */
#define I2C_DEVICE "/dev/i2c-0"
#define I2C_SLAVE 0x0703
#define DSPIC_I2C_ADDR 0x8
#define DSPIC_SWITCH_REG 220

void MmapImageTranscriber::process_image (const void * p){
    fputc ('.', stdout);
    fflush (stdout);
    sensors->setImage(static_cast<const unsigned char*>(p));
}

MmapImageTranscriber::MmapImageTranscriber(shared_ptr<Synchro> synchro,
                                           shared_ptr<Sensors> s,
                                           ALPtr<ALBroker> broker)
    : ThreadedImageTranscriber(s,synchro,"MmapImageTranscriber"),
      log(), camera_active(false), dev_name("/dev/video0"), fd(-1)
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
    init_cameras();
    if (open_device()){
        camera_active = true;
        init_device();
    }
    else {
        camera_active = false;
        std::cout << "\tCamera is inactive!" << std::endl;
    }

#endif
}

MmapImageTranscriber::~MmapImageTranscriber() {
    if (camera_active){
        stop_capturing();
        uninit_device();
        close_device();
        close_i2c();
    }
}

void MmapImageTranscriber::run() {
    Thread::running = true;
    Thread::trigger->on();

    while (Thread::running) {
        if (camera_active){
            //start timer
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO (&fds);
            FD_SET (fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select (fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r) {
                if (EINTR == errno)
                    continue;

                errno_exit ("select");
            }

            if (0 == r) {
                fprintf (stdout, "select timeout\n");
            }

            if (read_frame ()){

            }
        }

    }

    Thread::trigger->off();
}

int MmapImageTranscriber::start() {
#ifdef USE_VISION
    if(!camera_active) {
        if (open_device()){
            camera_active = true;
            init_device();
        }
        else {
            camera_active = false;
            std::cout << "\tCamera is inactive!" << std::endl;
        }
    }

    start_capturing();
#endif

    return Thread::start();
}

void MmapImageTranscriber::stop() {
    std::cout << "Stopping MmapImageTranscriber" << std::endl;
    running = false;
#ifdef USE_VISION
    if(camera_active){
        stop_capturing();
        uninit_device();
        close_device();
        close_i2c();
    }

#endif

    Thread::stop();
}

void MmapImageTranscriber::start_capturing() {
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = i;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                errno_exit ("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            errno_exit ("VIDIOC_STREAMON");

        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_USERPTR;
            buf.index       = i;
            buf.m.userptr	= (unsigned long) buffers[i].start;
            buf.length      = buffers[i].length;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                errno_exit ("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            errno_exit ("VIDIOC_STREAMON");

        break;
    }
}

void MmapImageTranscriber::stop_capturing() {
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
            errno_exit ("VIDIOC_STREAMOFF");
        break;
    }
}

int MmapImageTranscriber::read_frame (void){
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit ("read");
            }
        }

        process_image (buffers[0].start);

        break;

    case IO_METHOD_MMAP:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit ("VIDIOC_DQBUF");
            }
        }

        assert (buf.index < n_buffers);

        process_image (buffers[buf.index].start);

        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            errno_exit ("VIDIOC_QBUF");

        break;

    case IO_METHOD_USERPTR:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit ("VIDIOC_DQBUF");
            }
        }

        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long) buffers[i].start
                && buf.length == buffers[i].length)
                break;

        assert (i < n_buffers);

        process_image ((void *) buf.m.userptr);

        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            errno_exit ("VIDIOC_QBUF");

        break;
    }

    return 1;
}

void MmapImageTranscriber::init_read (unsigned int buffer_size){
    buffers = static_cast<buffer*>(calloc (1, sizeof (*buffers)));

    if (!buffers) {
        fprintf (stdout, "Out of memory\n");
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc (buffer_size);

    if (!buffers[0].start) {
        fprintf (stdout, "Out of memory\n");
    }
}

void MmapImageTranscriber::init_mmap (void){
    struct v4l2_requestbuffers req;

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf (stdout, "%s does not support "
                     "memory mapping\n", dev_name);
        } else {
            errno_exit ("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf (stdout, "Insufficient buffer memory on %s\n",
                 dev_name);
    }

    buffers = static_cast<buffer*>(calloc (req.count, sizeof (*buffers)));

    if (!buffers) {
        fprintf (stdout, "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
            errno_exit ("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
            mmap (NULL /* start anywhere */,
                  buf.length,
                  PROT_READ | PROT_WRITE /* required */,
                  MAP_SHARED /* recommended */,
                  fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            errno_exit ("mmap");
    }
}

void MmapImageTranscriber::init_userp (unsigned int buffer_size){
    struct v4l2_requestbuffers req;
    unsigned int page_size;

    page_size = getpagesize ();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf (stdout, "%s does not support "
                     "user pointer i/o\n", dev_name);
        } else {
            errno_exit ("VIDIOC_REQBUFS");
        }
    }

    buffers = static_cast<buffer*>(calloc (4, sizeof (*buffers)));

    if (!buffers) {
        fprintf (stdout, "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                             buffer_size);

        if (!buffers[n_buffers].start) {
            fprintf (stdout, "Out of memory\n");
        }
    }
}

void MmapImageTranscriber::init_device (void){
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;
    std::cout << "init'ing device" << std::endl;
    if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf (stdout, "%s is no V4L2 device\n",
                     dev_name);
        } else {
            errno_exit ("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf (stdout, "%s is no video capture device\n",
                 dev_name);
    }

    switch (io) {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
            fprintf (stdout, "%s does not support read i/o\n",
                     dev_name);
        }

        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf (stdout, "%s does not support streaming i/o\n",
                     dev_name);
        }

        break;
    }

    /* Select video input, video standard and tune here. */
    // GET video device information
    v4l2_std_id esid = esid0;
    if ( ioctl( fd, VIDIOC_S_STD, &esid )) {
        errno_exit ("VIDIOC_S_STD");
    }

    CLEAR (fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    fmt.fmt.pix.width = NAO_IMAGE_HEIGHT;
    fmt.fmt.pix.height = NAO_IMAGE_WIDTH;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

    if (-1 == ioctl (fd, VIDIOC_S_FMT, &fmt)){
        errno_exit ("VIDIOC_S_FMT");
        // exit (EXIT_FAILURE);
    }

    /* Note VIDIOC_S_FMT may change width and height. */

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    CLEAR (cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        crop.c.width = NAO_IMAGE_WIDTH;
        crop.c.height = NAO_IMAGE_HEIGHT;
        std::cout << "width: " << crop.c.width << std::endl;
        std::cout << "height: " << crop.c.height << std::endl;
        std::cout << "left: " << crop.c.left << std::endl;
        std::cout << "top: " << crop.c.top << std::endl;

        if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    // set fps
    struct v4l2_streamparm parm;
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if( 0 != xioctl(fd, VIDIOC_G_PARM, &parm)){
        perror("VIDIO_G_PARM");
    }

    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = VISION_FPS;
    parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    if (0 > xioctl(fd, VIDIOC_S_PARM, &parm)){
        perror("VIDIO_G_PARM");
    }


    switch (io) {
    case IO_METHOD_READ:
        init_read (fmt.fmt.pix.sizeimage);
        break;

    case IO_METHOD_MMAP:
        init_mmap ();
        break;

    case IO_METHOD_USERPTR:
        init_userp (fmt.fmt.pix.sizeimage);
        break;
    }
}

void MmapImageTranscriber::uninit_device (void){
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        free (buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap (buffers[i].start, buffers[i].length))
                errno_exit ("munmap");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free (buffers[i].start);
        break;
    }

    free (buffers);
}

bool MmapImageTranscriber::open_device (void){
    struct stat st;
    std::cout<<"opening device" << std::endl;
    if (-1 == stat (dev_name, &st)) {
        fprintf (stdout, "Cannot identify '%s': %d, %s\n",
                 dev_name, errno, strerror (errno));
        return false;
    }

    if (!S_ISCHR (st.st_mode)) {
        fprintf (stdout, "%s is no device\n", dev_name);
        return false;
    }

    fd = open (dev_name, O_RDWR /* required */ + O_NONBLOCK, 0);
    //fd = open (dev_name, O_RDWR /* required */, 0);

    if (-1 == fd) {
        fprintf (stdout, "Cannot open '%s': %d, %s\n",
                 dev_name, errno, strerror (errno));
        return false;
    }

    return true;
}

void MmapImageTranscriber::close_device (void){
    if (-1 == close (fd))
        errno_exit ("close");

    fd = -1;
}

bool MmapImageTranscriber::open_i2c (void){
    i2cBus = open("/dev/i2c-0", O_RDWR);
    if (-1 == i2cBus){
        return false;
    }
    return true;
}

void MmapImageTranscriber::close_i2c (void){
    close (i2cBus);
    i2cBus = -1;
}

void MmapImageTranscriber::init_cameras (void) {
    open_i2c();
    set_camera_top();
    setCameraSettings();
    set_camera_bottom();
    setCameraSettings();
    close_i2c();
}

void MmapImageTranscriber::set_camera_bottom (void){
    if( 0 > ioctl( i2cBus, I2C_SLAVE, DSPIC_I2C_ADDR) ) {
        std::cout << " : Can't connect I2C to dsPIC" << std::endl;
    }

    static unsigned char cmd [2] = {0, 0};
    cmd[0] = 0x02;
    if( 0 > i2c_smbus_write_block_data(i2cBus, DSPIC_SWITCH_REG, 1, cmd)){
        std::cout << "error setting bottom camera" << std::endl;
    }
}

void MmapImageTranscriber::set_camera_top (void){
    if( 0 > ioctl( i2cBus, I2C_SLAVE, DSPIC_I2C_ADDR) ) {
        std::cout << " : Can't connect I2C to dsPIC" << std::endl;
    }

    static unsigned char cmd [2] = {0, 0};
    cmd[0] = 0x01;
    if( 0 > i2c_smbus_write_block_data(i2cBus, DSPIC_SWITCH_REG, 1, cmd)){
        std::cout << "error setting top camera" << std::endl;
    }
}

void MmapImageTranscriber::setCameraSettings(void) {
    //struct v4l2_queryctrl queryctrl;
    struct v4l2_control control;
    CLEAR(control);

    open_device();
    /**
     * macro to set a control on the camera.  reports errors but does not stop.
     *
     * @param controlId    the id of the control
     * @param controlValue the value to set controlId to
     */
#define setControl(controlId, controlValue)                             \
    {                                                                   \
        control.id    = controlId;                                      \
        control.value = controlValue;                                   \
        if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control) && errno != ERANGE){ \
            errno_exit("set control error");                            \
        }                                                               \
    }

    //setControl(V4L2_CID_SATURATION,         128);
    setControl(V4L2_CID_HUE,                0);
    setControl(V4L2_CID_AUDIO_MUTE,         false);//Auto Exposure
    setControl(V4L2_CID_AUTO_WHITE_BALANCE, false);
    //setControl(V4L2_CID_EXPOSURE,           504);
    setControl(V4L2_CID_AUTOGAIN,           false);
    for(unsigned int controlIndex = 0; controlIndex < NUM_CONTROLS;
        ++controlIndex){
        setControl(controlIds[controlIndex], controlValues[0][controlIndex]);
    }
    close_device();
}

void MmapImageTranscriber::errno_exit (const char * s){
    fprintf (stdout, "%s error %d, %s\n",
             s, errno, strerror (errno));
    fflush(stdout);
}

int MmapImageTranscriber::xioctl (int fd, int request, void * arg){
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

void MmapImageTranscriber::releaseImage(void){
}
