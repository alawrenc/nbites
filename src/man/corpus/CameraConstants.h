#ifndef CAMERA_CONSTANTS_H
#define CAMERA_CONSTANS_H
/*
Camera setting constants fro use by image transcribers
 */
static const int TOP_CAMERA = 0;
static const int BOTTOM_CAMERA = 1;

// Camera setup information
static const int CAMERA_SLEEP_TIME = 200;
static const int CAM_PARAM_RETRIES = 3;

#if NAO_IMAGE_SIZE == VGA
#define esid0 0x08000000UL
#elif NAO_IMAGE_SIZE == QVGA
#define esid0 0x04000000UL
#else
#  error Undefined Nao image type
#endif


#define OC_PARAMS
#ifdef OC_PARAMS
static const int DEFAULT_CAMERA_RESOLUTION = 14;
static const int DEFAULT_CAMERA_FRAMERATE = 30;
static const int DEFAULT_CAMERA_BUFFERSIZE = 16;
// Color Settings
// Gain: 26 / Exp: 83
// Gain: 28 / Exp: 60
// Gain: 35 / Exp: 40
static const int DEFAULT_CAMERA_AUTO_GAIN = 0; // AUTO GAIN OFF
static const int DEFAULT_CAMERA_GAIN = 30;
static const int DEFAULT_CAMERA_AUTO_WHITEBALANCE = 0; // AUTO WB OFF
static const int DEFAULT_CAMERA_BLUECHROMA = 127;
static const int DEFAULT_CAMERA_REDCHROMA = 67;
static const int DEFAULT_CAMERA_BRIGHTNESS = 154;
static const int DEFAULT_CAMERA_CONTRAST = 90;
static const int DEFAULT_CAMERA_SATURATION = 152;
static const int DEFAULT_CAMERA_HUE = 0;
// Lens correction
static const int DEFAULT_CAMERA_LENSX = 0;
static const int DEFAULT_CAMERA_LENSY = 0;
// Exposure length
static const int DEFAULT_CAMERA_AUTO_EXPOSITION = 0; // AUTO EXPOSURE OFF
static const int DEFAULT_CAMERA_EXPOSURE = 60;
// Image orientation
static const int DEFAULT_CAMERA_HFLIP = 0;
static const int DEFAULT_CAMERA_VFLIP = 0;
#else
// Default Camera Settings
// Basic Settings
static const int DEFAULT_CAMERA_RESOLUTION = 14;
static const int DEFAULT_CAMERA_FRAMERATE = 30;
static const int DEFAULT_CAMERA_BUFFERSIZE = 16;
// Color Settings
// Gain: 26 / Exp: 83
// Gain: 28 / Exp: 60
// Gain: 35 / Exp: 40
static const int DEFAULT_CAMERA_AUTO_GAIN = 0; // AUTO GAIN OFF
static const int DEFAULT_CAMERA_GAIN = 30;
static const int DEFAULT_CAMERA_AUTO_WHITEBALANCE = 0; // AUTO WB OFF
static const int DEFAULT_CAMERA_BLUECHROMA = 127;
static const int DEFAULT_CAMERA_REDCHROMA = 67;
static const int DEFAULT_CAMERA_BRIGHTNESS = 154;
static const int DEFAULT_CAMERA_CONTRAST = 90;
static const int DEFAULT_CAMERA_SATURATION = 152;
static const int DEFAULT_CAMERA_HUE = 0;
// Lens correction
static const int DEFAULT_CAMERA_LENSX = 0;
static const int DEFAULT_CAMERA_LENSY = 0;
// Exposure length
static const int DEFAULT_CAMERA_AUTO_EXPOSITION = 0; // AUTO EXPOSURE OFF
static const int DEFAULT_CAMERA_EXPOSURE = 60;
// Image orientation
static const int DEFAULT_CAMERA_HFLIP = 0;
static const int DEFAULT_CAMERA_VFLIP = 0;
#endif
#endif
