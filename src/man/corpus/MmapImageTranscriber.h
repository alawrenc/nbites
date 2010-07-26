#ifndef MMAPIMAGE_TRANSCRIBER_H
#define MMAPIMAGE_TRANSCRIBER_H

#include "almemoryproxy.h"
#include "albroker.h"
#include "alptr.h"
#include "alloggerproxy.h"

#include "ThreadedImageTranscriber.h"
#include "synchro.h"
#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

class MmapImageTranscriber : public ThreadedImageTranscriber {
 public:
    MmapImageTranscriber(boost::shared_ptr<Synchro> synchro,
                       boost::shared_ptr<Sensors> s,
                       AL::ALPtr<AL::ALBroker> broker);
    virtual ~MmapImageTranscriber();

 private:
    MmapImageTranscriber(const MmapImageTranscriber &other);
    void operator= (const MmapImageTranscriber &other);

 public:

    int start();
    void run();
    void stop();
    void releaseImage();

 private: // helper methods
    bool registerCamera();
    void initCameraSettings();
    void waitForImage();
    void initializeBuffers();
    void start_streaming();
    void stop_streaming();
    int xioctl (int fd, int request, void * arg);

 private: // member variables
    // Interfaces/Proxies to robot

    // stream descriptor for video camera
    int sd;
    AL::ALPtr<AL::ALLoggerProxy> log;
    bool camera_active;

    static const unsigned int REQUIRED_BUFFERS = 4;
    struct image_buffer{
        unsigned char *start;
        size_t length;
    };
    image_buffer buffers [REQUIRED_BUFFERS];
    v4l2_buffer current_frame;
    v4l2_buffer last_frame;
};

#endif
