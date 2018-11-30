////////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//

//
// The code has been contributed by Arkadiusz Raj on 2016 Oct
//

#include "precomp.hpp"

#ifdef HAVE_ARAVIS_API

#include <arv.h>
#include "master.h"
#include "pvcam.h"


//
// This file provides wrapper for using Aravis SDK library to access GigE Vision cameras.
// Aravis library (version 0.4 or 0.6) shall be installed else this code will not be included in build.
//
// To include this module invoke cmake with -DWITH_ARAVIS=ON
//
// Please obvserve, that jumbo frames are required when high fps & 16bit data is selected.
// (camera, switches/routers and the computer this software is running on)
//
// Basic usage: VideoCapture cap(CAP_ARAVIS + <camera id>);
//
// Supported properties:
//  read/write
//      CAP_PROP_AUTO_EXPOSURE(0|1)
//      CAP_PROP_EXPOSURE(t), t in seconds
//      CAP_PROP_BRIGHTNESS (ev), exposure compensation in EV for auto exposure algorithm
//      CAP_PROP_GAIN(g), g >=0 or -1 for automatic control if CAP_PROP_AUTO_EXPOSURE is true
//      CAP_PROP_FPS(f)
//      CAP_PROP_FOURCC(type)
//      CAP_PROP_BUFFERSIZE(n)
//  read only:
//      CAP_PROP_POS_MSEC
//      CAP_PROP_FRAME_WIDTH
//      CAP_PROP_FRAME_HEIGHT
//
//  Supported types of data:
//      video/x-raw, fourcc:'GREY'  -> 8bit, 1 channel
//      video/x-raw, fourcc:'Y800'  -> 8bit, 1 channel
//      video/x-raw, fourcc:'Y12 '  -> 12bit, 1 channel
//      video/x-raw, fourcc:'Y16 '  -> 16bit, 1 channel
//      video/x-raw, fourcc:'GRBG'  -> 8bit, 1 channel
//

#define MODE_GREY   CV_FOURCC_MACRO('G','R','E','Y')
#define MODE_Y800   CV_FOURCC_MACRO('Y','8','0','0')
#define MODE_Y12    CV_FOURCC_MACRO('Y','1','2',' ')
#define MODE_Y16    CV_FOURCC_MACRO('Y','1','6',' ')
#define MODE_GRBG   CV_FOURCC_MACRO('G','R','B','G')

#define CLIP(a,b,c) (cv::max(cv::min((a),(c)),(b)))



#define      CAP_PVCAM_USB  5000
#define      CAP_PVCAM_PCIE  5001
#define      CAP_PVCAM_1394  5002


#define MAX_PORT_DESC_LEN 80
struct speed_type {
    int16 port_index;
    char port_desc[MAX_PORT_DESC_LEN];
    int16 spdtab_index;
    uns16 pix_time;
    uns16 bit_depth;
    uns16 max_gain_index;
};

#define MAX_SPEEDS 16
struct speed_table_type {
    uns32 speed_count;
    struct speed_type speeds[MAX_SPEEDS];
};

typedef struct SampleContext
{
   int myData1;
   int myData2;
}
SampleContext;
/********************* Capturing video from camera via PVCAM *********************/

class CvCaptureCAM_PVCAM : public CvCapture
{
public:
    CvCaptureCAM_PVCAM();
    virtual ~CvCaptureCAM_PVCAM()
    {
        close();
    }

    virtual bool open(int);
    virtual void close();
    virtual double getProperty(int) const;
    virtual bool setProperty(int, double);
    virtual bool grabFrame();
    virtual IplImage* retrieveFrame(int);
    virtual int getCaptureDomain()
    {
        return CaptureDomain;
    }

protected:
    bool create(int);
    bool init_buffers();

    void stopCapture();
    bool startCapture();

    bool getDeviceNameById(int id, std::string &device);

    void autoExposureControl(IplImage*);


    rs_bool init_pvcam();
    rs_bool uninit_pvcam();
    rs_bool open_cam(int wIndex);
    rs_bool close_cam();
    rs_bool start_capture(double exp_time_sec);
    rs_bool stop_capture();
    rs_bool is_equare(rgn_type * current ,rgn_type * old);
    rs_bool speeds_show_table (int16 hcam,struct speed_table_type *speed_table);
    rs_bool speeds_get_table(int16 hcam, struct speed_table_type *speed_table);
    rs_bool speeds_set_speed(int16 hcam, uns16 index,
        struct speed_table_type speed_table);


    ArvCamera       *camera;                // Camera to control.
    ArvStream       *stream;                // Object for video stream reception.
    void            *framebuffer;           //

    unsigned int    payload;                // Width x height x Pixel width.

    int             widthMin;               // Camera sensor minium width.
    int             widthMax;               // Camera sensor maximum width.
    int             heightMin;              // Camera sensor minium height.
    int             heightMax;              // Camera sensor maximum height.
    bool            fpsAvailable;
    double          fpsMin;                 // Camera minium fps.
    double          fpsMax;                 // Camera maximum fps.
    bool            gainAvailable;
    double          gainMin;                // Camera minimum gain.
    double          gainMax;                // Camera maximum gain.
    bool            exposureAvailable;
    double          exposureMin;            // Camera's minimum exposure time.
    double          exposureMax;            // Camera's maximum exposure time.

    bool            controlExposure;        // Flag if automatic exposure shall be done by this SW
    double          exposureCompensation;
    bool            autoGain;
    double          targetGrey;             // Target grey value (mid grey))

    gint64          *pixelFormats;
    guint           pixelFormatsCnt;
    int             CaptureDomain;      //= CAP_PVCAM_PCIE;


    int             num_buffers;            // number of payload transmission buffers

    ArvPixelFormat  pixelFormat;            // pixel format

    unsigned int             xoffset;                // current frame region x offset
    unsigned int             yoffset;                // current frame region y offset
    unsigned int             width;                  // current frame width of frame
    unsigned int             height;                 // current frame height of image

    double          fps;                    // current value of fps
    double          exposure;               // current value of exposure time
    double          gain;                   // current value of gain
    double          midGrey;                // current value of mid grey (brightness)
    unsigned        frameID;                // current frame id
    unsigned        prevFrameID;
    IplImage        *frame;                 // local frame copy
    unsigned char   * fake_buffer; 

    uns16 *frameAddress;
    uns8 *circBufferInMemory;
    int16 hcam;
    rs_bool opensuccess;
    rs_bool startsuccess;
    uns32 exposureBytes;
    struct speed_table_type speed_table;  
    char msg[ERROR_MSG_LEN];
    char cam_name[CAM_NAME_LEN];
    uns16 max_width;
    uns16 max_height;
    uns16 frames;
    uns16 depth;
    rgn_type g_Region;
    rgn_type g_Region_old;
    SampleContext dataContext;
    double   exposure_old;           
    unsigned char * circBuffer;
    unsigned int camera_detected_width ;
    unsigned int camera_detected_height ;
    uns16 current_speed;
    uns16 min_speed;
    uns16 max_speed;
    uns16 min_gain;
    uns16 max_gain;
    uns16 current_gain;
    uns16 old_gain;
    uns16 old_speed;

    uns16 clear_cycles;
    uns16 clear_cycles_avail; 
    uns16 clear_mode;
    uns16 clear_mode_avail; 
    uns16 clear_expose_out_mode;
    uns16 clear_expose_out_mode_avail;
    uns32 temperature; 
    uns16 old_clear_cycles; 
    uns16 old_clear_mode;  
    uns16 old_clear_expose_out_mode;

public:
    pthread_mutex_t count_mutex ;   
    pthread_cond_t  condition_var; 
};








rs_bool CvCaptureCAM_PVCAM::speeds_show_table (int16 hcam,struct speed_table_type *speed_table)
{
    char msg[ERROR_MSG_LEN];
    uns32 speed_count = speed_table->speed_count;
    printf("speed_count  %d\n",speed_count);
    for(uns32 i =0; i<speed_count;i++)
    {
         printf("   speed_index  %d\n",i);
         printf("       port_index  %d\n",speed_table->speeds[i].port_index);
         printf("       spdtab_index  %d\n",speed_table->speeds[i].spdtab_index);
         printf("       max_gain_index  %d\n",speed_table->speeds[i].max_gain_index);
    }

    min_speed = 0;
    if(speed_count > 0)
    {
       max_speed = speed_count -1;
    }

    uns16 port_current_index;
    if (PV_OK != pl_get_param(hcam, PARAM_READOUT_PORT, ATTR_CURRENT,
            (void *)&port_current_index)) {
        printf("Can't get readout port count, error\n");
        return PV_FAIL;
    }

    uns16 speed_current_index;
    if (PV_OK != pl_get_param(hcam, PARAM_SPDTAB_INDEX, ATTR_CURRENT,
                (void *)&speed_current_index)) {
            printf(
                    "Can't get current speed table index, error \n");
            return PV_FAIL;
        }
    printf("speed_current_index  %d\n",speed_current_index);
    current_speed = speed_current_index;

    uns16 gain_current_index;
    if (PV_OK != pl_get_param(hcam, PARAM_GAIN_INDEX, ATTR_CURRENT,
                    (void *)&gain_current_index)) {
                printf(
                        "Can't get current gain index, error \n");
                return PV_FAIL;
     }
     printf("gain_current_index  %d\n",gain_current_index);
     current_gain = gain_current_index;

     min_gain = 1;
     max_gain = speed_table->speeds[current_speed].max_gain_index;


    return true;


}



rs_bool CvCaptureCAM_PVCAM::speeds_get_table(int16 hcam, struct speed_table_type *speed_table)
{
    uns32 port_count;
    char port_desc[MAX_PORT_DESC_LEN];
    int32 port_val;
    uns32 spdtab_count;

    uns32 enum_str_len;
    int32 enum_val;
    char *enum_str = NULL;

    uns16 pix_time;
    uns16 bit_depth;
    uns16 max_gain_index;

    int16 pi;
    int16 si;
    char msg[ERROR_MSG_LEN];

    int index = 0;

    if (NULL == speed_table) {
        printf( "Uninitialized speed table\n");
        return PV_FAIL;
    }

    memset(speed_table, 0, sizeof(*speed_table));

    if (PV_OK != pl_get_param(hcam, PARAM_READOUT_PORT, ATTR_COUNT,
            (void *)&port_count)) {
        printf("Can't get readout port count, error \n");
        return PV_FAIL;
    }
  
    for (pi = 0; pi < (int16)port_count; pi++) {
        if (PV_OK != pl_set_param(hcam, PARAM_READOUT_PORT, (void *)&pi)) {
            printf("Can't set readout port, error\n");
            goto err;
        }
        speed_table->speeds[index].port_index = pi;

        if (PV_OK != pl_enum_str_length(hcam, PARAM_READOUT_PORT, pi,
                &enum_str_len)) {
            printf(
                    "Can't get readout port description lenght, error \n");
            goto err;
        }
        enum_str = (char *)calloc(sizeof(char), enum_str_len + 1);
        if (NULL == enum_str) {
            printf(
                    "Can't allocate memory for readout portdescription\n");
            goto err;
        }
        if (PV_OK != pl_get_enum_param(hcam, PARAM_READOUT_PORT, pi,
                (int32 *)&enum_val, enum_str, enum_str_len)) {
            printf( "Can't get readout port internals, error \n");
            goto err;
        }

        if (PV_OK != pl_get_param(hcam, PARAM_SPDTAB_INDEX, ATTR_COUNT,
                (void *)&spdtab_count)) {
        
            printf(
                    "Can't get readout port speed count, error\n");
            goto err;
        }
      
        for (si = 0; si < (int16)spdtab_count; si++) {
            snprintf(speed_table->speeds[index].port_desc, MAX_PORT_DESC_LEN, "%s", enum_str);

            if (PV_OK != pl_set_param(hcam, PARAM_SPDTAB_INDEX, (void *)&si)) {
               
                printf(
                        "Can't set readout port speed index, error \n");
                goto err;
            }
            speed_table->speeds[index].spdtab_index = si;

            if (PV_OK != pl_get_param(hcam, PARAM_PIX_TIME, ATTR_CURRENT,
                    (void *)&speed_table->speeds[index].pix_time)) {
              
                printf(
                        "Can't get readout port pixel time, error \n");
                goto err;
            }

            if (PV_OK != pl_get_param(hcam, PARAM_BIT_DEPTH, ATTR_CURRENT,
                    (void *)&speed_table->speeds[index].bit_depth)) {
                
                printf(
                        "Can't get readout port bit depth, error \n");
                goto err;
            }

            if (PV_OK != pl_get_param(hcam, PARAM_GAIN_INDEX, ATTR_MAX,
                    (void *)&speed_table->speeds[index].max_gain_index)) {
               
                printf(
                        "Can't get readout port max. gain index, error \n");
                goto err;
            }
             

            index++;
            speed_table->speed_count = index;
        }

        free(enum_str);
        enum_str = NULL;
    }

    return PV_OK;

err:
    memset(speed_table, 0, sizeof(*speed_table));
    free(enum_str);
    return PV_FAIL;
}

rs_bool CvCaptureCAM_PVCAM::speeds_set_speed(int16 hcam, uns16 index,
        struct speed_table_type speed_table)
{
    char msg[ERROR_MSG_LEN];

    if (index >= speed_table.speed_count || index >= MAX_SPEEDS) {
        printf("Speed index is out of bound\n");
        return PV_FAIL;
    }

    if (speed_table.speed_count >= MAX_SPEEDS) {
        printf("Speed table count is out of bound\n");
        return PV_FAIL;
    }

    if (PV_OK != pl_set_param(hcam, PARAM_READOUT_PORT,
            (void *)&speed_table.speeds[index].port_index)) {
        
        printf("Can't set readout port, error \n");
        return PV_FAIL;
    }

    if (PV_OK != pl_set_param(hcam, PARAM_SPDTAB_INDEX,
            (void *)&speed_table.speeds[index].spdtab_index)) {

        printf("Can't set readout port speed index, error \n");
        return PV_FAIL;
    }

    return PV_OK;
}







void PV_DECL NewFrameHandler(FRAME_INFO *pFrameInfo, void *context)
{
    CvCaptureCAM_PVCAM * this_instance = (CvCaptureCAM_PVCAM *)context;
   // printf("NewFrameHandler cond signal\n");
    pthread_cond_signal( &(this_instance->condition_var) );  
}




rs_bool CvCaptureCAM_PVCAM::init_pvcam()
{
    if (PV_OK != pl_pvcam_init()) {
        const int16 error = pl_error_code();
        pl_error_message(error, msg);
        printf("Can't initialize PVCAM, error %d (%s)\n", error, msg);
        return FALSE;
    }
    else
    {
        printf("pl_pvcam_init success\n");
    }
    return TRUE;
}


rs_bool CvCaptureCAM_PVCAM::uninit_pvcam()
{
    if (PV_OK != pl_pvcam_uninit()) {
        const int16 error = pl_error_code();
        pl_error_message(error, msg);
        printf("Can't uninitialize PVCAM, error %d (%s)\n", error,
                msg);
        return FALSE;
    }
    else
    {
        printf("pl_pvcam_uninit success\n");
    }
    return TRUE;
}




rs_bool CvCaptureCAM_PVCAM::open_cam(int wIndex)
{
    opensuccess = FALSE;
  
    char cameraName[CAM_NAME_LEN] = "\0";
    int16 cameraCount = 0;
    pl_cam_get_total(&cameraCount);

    if (cameraCount == 0)
    {
        printf("No PVCAM camera found error!!\n");
        return FALSE;
    }
    if (wIndex == CAP_PVCAM_USB)
    {
        strcpy(cameraName, "USB");
	CaptureDomain = CAP_PVCAM_USB;
    }
    else if (wIndex == CAP_PVCAM_PCIE)
    {
        strcpy(cameraName, "PCIE");
	CaptureDomain = CAP_PVCAM_PCIE;
    }
    else if (wIndex == CAP_PVCAM_1394)
    {
        strcpy(cameraName, "1394");
	CaptureDomain = CAP_PVCAM_1394;
    }
    else if (wIndex == 0)
    {
      
    }
    else
    {
        return FALSE;
    }
    bool  find_cam = false;
    for (int i = 0; i<cameraCount; i++)
    {
        pl_cam_get_name(i, cam_name);
        if (strstr(cam_name ,cameraName))
        {
            find_cam = true;
            break;
        }
        else if (wIndex == 0)
        {
            find_cam = true;
            break;
        }
       
    }
    if (!find_cam)
    {
        return FALSE;
    }
 
    printf("cam_name %s\n", cam_name);

    if (strstr(cam_name ,"USB"))
    {
        CaptureDomain = CAP_PVCAM_USB;
           
    }
    else if (strstr(cam_name ,"PCIE"))
    {
        CaptureDomain = CAP_PVCAM_PCIE;          
    }
    else if (strstr(cam_name ,"1394"))
    {
        CaptureDomain = CAP_PVCAM_1394;         
    }

   

    if (PV_OK != pl_cam_open(cam_name, &hcam, OPEN_EXCLUSIVE)) {
            printf("Can't open camera '%s', error \n", cam_name);
        return FALSE;
    }
    else
    {
        printf("pl_cam_open success\n");
    }
    
  
    
/// test
    
    uns32 readvalue =0 ;

    if (PV_OK != pl_get_param(hcam, PARAM_CLEAR_CYCLES, ATTR_AVAIL,
            (void *)&readvalue)) {
        printf( "Can't get camera PARAM_CLEAR_CYCLES, error \n");
       
    }
    clear_cycles_avail = readvalue;
    printf("clear_cycles_avail  %d\n",clear_cycles_avail);

    if (PV_OK != pl_get_param(hcam, PARAM_CLEAR_CYCLES, ATTR_CURRENT,
            (void *)&readvalue)) {
        printf( "Can't get camera PARAM_CLEAR_CYCLES, error \n");
       
    }
    clear_cycles = readvalue;
    printf("clear_cycles  %d\n",clear_cycles);

    
    if (PV_OK != pl_get_param(hcam, PARAM_CLEAR_MODE, ATTR_AVAIL,
            (void *)&readvalue)) {
        printf( "Can't get camera PARAM_CLEAR_MODE, error \n");
       
    }
    clear_mode_avail = readvalue;
    printf("clear_mode_avail  %d\n",clear_mode_avail);


    if (PV_OK != pl_get_param(hcam, PARAM_CLEAR_MODE, ATTR_CURRENT,
            (void *)&readvalue)) {
        printf( "Can't get camera PARAM_CLEAR_MODE, error \n");
       
    }
    clear_mode = readvalue;
    printf("clear_mode  %d\n",clear_mode);


   
    
    if (PV_OK != pl_get_param(hcam, PARAM_EXPOSE_OUT_MODE, ATTR_AVAIL,
            (void *)&readvalue)) {
  
        printf( "Can't get camera PARAM_EXPOSE_OUT_MODE, error\n");
       
    }
    clear_expose_out_mode_avail = readvalue; 
    printf("clear_expose_out_mode_avail  %d\n",clear_expose_out_mode_avail);

   
    if (PV_OK != pl_get_param(hcam, PARAM_EXPOSE_OUT_MODE, ATTR_CURRENT,
            (void *)&clear_mode)) {
     
        printf( "Can't get camera PARAM_EXPOSE_OUT_MODE, error \n");
       
    }
    clear_expose_out_mode = clear_mode;
    printf("clear_expose_out_mode  %d\n",clear_expose_out_mode);

    if (PV_OK != pl_get_param(hcam, PARAM_TEMP, ATTR_CURRENT,
            (void *)&readvalue)) {
       
        printf( "Can't get camera PARAM_TEMP, error \n");
       
    }
    temperature = readvalue;
    printf("temperature  %d\n",temperature);

    if (PV_OK != pl_get_param(hcam, PARAM_SER_SIZE, ATTR_CURRENT,
            (void *)&max_width)) {
     
        printf( "Can't get camera resolution width, error\n");
        return FALSE;
    }
    camera_detected_width = max_width;
 
    if (PV_OK != pl_get_param(hcam, PARAM_PAR_SIZE, ATTR_CURRENT,
            (void *)&max_height)) {
  
        printf("Can't get camera resolution height, error \n");
        return FALSE;
    }
    camera_detected_height = max_height;

   if (PV_OK != speeds_get_table(hcam, &speed_table)) {
        return FALSE;
    }
    speeds_show_table (hcam,&speed_table);

    current_speed = 0;
    printf("speeds_set_speed speed = %d\n",current_speed);
    if (PV_OK != speeds_set_speed(hcam, current_speed, speed_table)) {
        return FALSE;
    }

    if (PV_OK != pl_get_param(hcam, PARAM_BIT_DEPTH, ATTR_CURRENT,
            (void *)&depth)) {
        printf("Can't get camera bit depth, error\n");
        return FALSE;
    }

    g_Region.s1 = 0;
    g_Region.s2 = max_width -1;
    g_Region.sbin = 1;
    g_Region.p1 = 0;
    g_Region.p2 = max_height -1;
    g_Region.pbin = 1;
    printf("g_Region s1 =%d , s2=%d ,sbin=%d ,p1=%d,p2=%d,pbin=%d \n",g_Region.s1,g_Region.s2,g_Region.sbin
                               ,g_Region.p1,g_Region.p2,g_Region.pbin);

  

    fake_buffer = (unsigned char *)malloc(camera_detected_width*camera_detected_height*2);
    if(fake_buffer == 0)
    {
       printf("Can't ALLOCATE MEMORY ERROR\n");
       return FALSE;
    }
    
    opensuccess = TRUE;
    return TRUE;
}

rs_bool CvCaptureCAM_PVCAM::close_cam()
{
   if(opensuccess)
   {
       if(fake_buffer != 0)
       {
          printf("free fake_buffer\n");
          free(fake_buffer);
          fake_buffer = 0;
       }
       if (PV_OK != pl_cam_close(hcam)) {
            const int16 error = pl_error_code();
            pl_error_message(error, msg);
            printf(
                     "Can't close camera '%s' with handler %d, error %d (%s)\n",
                      cam_name, (int)hcam, error, msg);
            return FALSE;
       }
       else
       {
           printf("pl_cam_close success\n");
       }
       
    }
    opensuccess = FALSE;
    return TRUE;
}

rs_bool CvCaptureCAM_PVCAM::is_equare(rgn_type * current ,rgn_type * old)
{

   if((current->s1 != old->s1)||(current->s2 != old->s2)||(current->sbin != old->sbin)||(current->p1 != old->p1)||(current->p2 != old->p2)||(current->pbin != old->pbin))
   {
      return false;
   }
   return true;


}


rs_bool CvCaptureCAM_PVCAM::start_capture(double exp_time_sec)
{
    
    uns32 exposureTime = (uns32)((double)exp_time_sec * (double)1000.0); // milliseconds

    const uns16 circBufferFrames = 2 ;  
    int16 bufferMode = CIRC_OVERWRITE;

    startsuccess = FALSE;

    if (PV_OK != speeds_set_speed(hcam, current_speed, speed_table)) {
           return FALSE;
    }

    if (PV_OK != pl_set_param(hcam, PARAM_GAIN_INDEX,(void *)&current_gain)) {
        			
        printf("Can't set PARAM_GAIN_INDEX, error \n");
        return PV_FAIL;
    }


    if(clear_mode_avail)
    {
       if (PV_OK != pl_set_param(hcam, PARAM_CLEAR_MODE,(void *)&clear_mode)) {
        			
        	printf("Can't set PARAM_CLEAR_MODE\n");
                return PV_FAIL;
          }
    }

    if(clear_cycles_avail)
    {
       if (PV_OK != pl_set_param(hcam, PARAM_CLEAR_CYCLES,(void *)&clear_cycles)) {
        			
        	printf("Can't set PARAM_CLEAR_CYCLES, error\n");
        	//return PV_FAIL;
       }
    }

    if(clear_expose_out_mode_avail)
    {
       if (PV_OK != pl_set_param(hcam, PARAM_EXPOSE_OUT_MODE,(void *)&clear_expose_out_mode)) {
        			
        	printf("Can't set PARAM_EXPOSE_OUT_MODE\n");
        	//return PV_FAIL;
       }
    }
    

    if (PV_OK != pl_cam_register_callback_ex3(hcam, PL_CALLBACK_EOF,
            (void *)NewFrameHandler, (void *)this))
    {
        printf("pl_cam_register_callback() error");
        return FALSE;
    }

    // Setup the acquisition
    if (PV_OK != pl_exp_setup_cont(hcam, 1, &g_Region, TIMED_MODE,
            exposureTime, &exposureBytes, bufferMode))
    {
        printf("pl_exp_setup_cont() error \n");
        return FALSE;
    }
    printf("Acquisition setup successful , circular frame counts  = %d  , exposure  %d(ms)\n",circBufferFrames,exposureTime);

    printf("g_Region s1 =%d , s2=%d ,sbin=%d ,p1=%d,p2=%d,pbin=%d \n",g_Region.s1,g_Region.s2,g_Region.sbin
                               ,g_Region.p1,g_Region.p2,g_Region.pbin);

    circBufferInMemory = (uns8 *) malloc( circBufferFrames * exposureBytes);
     
    if (circBufferInMemory == NULL)
    {
        printf("Unable to allocate memory\n");
        return FALSE;
    }

    if (PV_OK != pl_exp_start_cont(hcam, circBufferInMemory,
            circBufferFrames * exposureBytes ))
    {
        printf("pl_exp_start_cont() error \n");
        free(circBufferInMemory);
        circBufferInMemory = NULL;
        return FALSE;
    }

    g_Region_old.s1 = g_Region.s1;
    g_Region_old.s2 = g_Region.s2;
    g_Region_old.sbin = g_Region.sbin;
    g_Region_old.p1 = g_Region.p1;
    g_Region_old.p2 = g_Region.p2;
    g_Region_old.pbin = g_Region.pbin;

    exposure_old = exp_time_sec;
    old_gain = current_gain;
    old_speed = current_speed;

    old_clear_cycles = clear_cycles; 
    old_clear_mode = clear_mode ;  
    old_clear_expose_out_mode = clear_expose_out_mode;


    startsuccess = TRUE;
    return TRUE;
    
}

rs_bool CvCaptureCAM_PVCAM::stop_capture()
{

    if(startsuccess)
    {
       if (PV_OK != pl_exp_stop_cont(hcam, CCS_CLEAR))
       {
           printf("pl_exp_stop_cont error\n");
           return FALSE;
       }
       else
       {
           printf("pl_exp_stop_cont success\n");
       }
    }
    if(circBufferInMemory)
    {
        printf("free circBufferInMemory\n");
        free(circBufferInMemory);
        circBufferInMemory = NULL;
    }

    startsuccess = FALSE;
    return TRUE;

}




CvCaptureCAM_PVCAM::CvCaptureCAM_PVCAM()
{
    camera = NULL;
    stream = NULL;
    framebuffer = NULL;

    payload = 0;
    printf("CvCaptureCAM_PVCAM::CvCaptureCAM_PVCAM()\n");

    widthMin = widthMax = heightMin = heightMax = 0;
    xoffset = yoffset = width = height = 0;
    fpsMin = fpsMax = gainMin = gainMax = exposureMin = exposureMax = 0;
    controlExposure = false;
    exposureCompensation = 0;
    targetGrey = 0;
    frameID = prevFrameID = 0;

    num_buffers = 1;
    frame = NULL;
    frameAddress = 0;
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
    condition_var = PTHREAD_COND_INITIALIZER;
    circBufferInMemory = 0;
    hcam = 0;
    opensuccess = FALSE;
    startsuccess = FALSE;
    exposureBytes = 0;
    current_speed = 0;
    min_speed = 0;
    max_speed = 0;
    min_gain = 1;
    max_gain = 1;
    current_gain = 1;
    old_gain = 1;
    old_speed = 0;
    fake_buffer = 0;
    clear_cycles = 0;
    clear_cycles_avail = 0; 
    clear_mode = 0;
    clear_mode_avail = 0; 
    clear_expose_out_mode = 0;
    clear_expose_out_mode_avail = 0;
    temperature = 0; 
}

void CvCaptureCAM_PVCAM::close()
{
    printf("CvCaptureCAM_PVCAM::close()\n");
    stop_capture();
    close_cam();
    uninit_pvcam();
    if(fake_buffer != 0)
    {
       printf("free fake_buffer\n");
       free(fake_buffer);
       fake_buffer = 0;
    }

}

bool CvCaptureCAM_PVCAM::getDeviceNameById(int id, std::string &device)
{
    return TRUE;    
}

bool CvCaptureCAM_PVCAM::create( int index )
{
    std::string deviceName;
    printf("CvCaptureCAM_PVCAM::create\n");
    if(!getDeviceNameById(index, deviceName))
        return false;

    init_pvcam();
    return open_cam(index);
}

bool CvCaptureCAM_PVCAM::init_buffers()
{

    printf("CvCaptureCAM_PVCAM::init_buffers\n");
    return true;
}

bool CvCaptureCAM_PVCAM::open( int index )
{
    printf("CvCaptureCAM_PVCAM::open\n");
    if(create(index)) {
        gainAvailable = true;
        fpsAvailable = false;
        exposureAvailable = true;

        gain = current_gain;
        exposure = 0.010 ;

        xoffset = 0;
        yoffset = 0;
        width = max_width;
        height = max_height;
        printf("start xoffset %d ,yoffset %d,width %d,height %d\n",xoffset,yoffset,width,height);
        printf("exposure %f ,current_speed %d, current_gain %d\n",exposure,current_speed,current_gain);
        return true; //startCapture();
    }
    return false;
}




bool CvCaptureCAM_PVCAM::grabFrame()
{
    // remove content of previous frame
    framebuffer = NULL;
   // printf("CvCaptureCAM_PVCAM::grabFrame\n");


    if((g_Region.s2 > (camera_detected_width-1)) || (g_Region.p2 > (camera_detected_height-1))||(g_Region.s1 > g_Region.s2) ||(g_Region.p1 > g_Region.p2))
    {
         printf("ROI setting is invalid and out of range , stop capture capture\n");
         printf("g_Region s1 =%d , s2=%d ,sbin=%d ,p1=%d,p2=%d,pbin=%d \n",g_Region.s1,g_Region.s2,g_Region.sbin
                               ,g_Region.p1,g_Region.p2,g_Region.pbin);
         stop_capture();
    }

    if(!startsuccess)
    {
       startCapture();
    }

    
    if((!is_equare(&g_Region ,&g_Region_old)) || (exposure_old != exposure) || (old_gain != current_gain)|| (old_speed !=    current_speed) || (old_clear_cycles != clear_cycles) || (old_clear_mode != clear_mode)|| (old_clear_expose_out_mode != clear_expose_out_mode))
    {

         printf("Parameters changed, Re-start capture\n");
         stop_capture();
         usleep(100000);
         start_capture(exposure);
         usleep(100000);

    }
    

     while(startsuccess) {


         pthread_mutex_lock( &count_mutex );
         pthread_cond_wait( &condition_var, &count_mutex );
       
       	 if (PV_OK != pl_exp_get_latest_frame(hcam, (void **)&frameAddress))
         {
            //printf(
            //        "pl_exp_get_latest_frame() error\n");
            pthread_mutex_unlock( &count_mutex );
            continue;
         }
                     
         if( frameAddress != 0)
         {
 	
            memcpy((unsigned char *)fake_buffer,(unsigned char *)frameAddress,exposureBytes);          
            framebuffer =  fake_buffer;
            unsigned short * resize_buffer = (unsigned short *)framebuffer;
            uns16 min_value = 0xffff;
            uns16 max_value = 0;
            uns16 temp_value = 0;
            float temp_float_value = 0.0;
            for (uns32 n = 0; n < exposureBytes / 2; n++)
            {
                if (resize_buffer[n] > max_value)
                {
                    max_value = resize_buffer[n];
                }
                if (resize_buffer[n] < min_value)
                {
                    min_value = resize_buffer[n];
                }
            }
          //  printf("max_value : %d   , min_value : %d , exposureBytes*2 %d\n", max_value, min_value,exposureBytes*2);
            for (uns32 n = 0; n < exposureBytes / 2; n++)
            {
                temp_float_value = (float)((float)(resize_buffer[n] - min_value) / (float)(max_value - min_value));
                temp_value = (uns16)((temp_float_value * (float)0xffff));
                resize_buffer[n] = temp_value;
            }
         }
         pthread_mutex_unlock( &count_mutex );
         return true;	
      
    }
    return false;
}

IplImage* CvCaptureCAM_PVCAM::retrieveFrame(int)
{
   // printf("CvCaptureCAM_PVCAM::retrieveFrame\n");
    if(framebuffer) {
        int depth = 0, channels = 0;
        unsigned int width_src = width/g_Region.sbin;
        unsigned int height_src = height/g_Region.pbin;
      
        depth = IPL_DEPTH_16U;
        channels = 1;


        if(depth && channels) {
            IplImage src;
            cvInitImageHeader( &src, cvSize( width_src, height_src ), depth, channels, IPL_ORIGIN_TL, 4 );

            cvSetData( &src, framebuffer, src.widthStep );
            if( !frame ||
                 frame->width != src.width ||
                 frame->height != src.height ||
                 frame->depth != src.depth ||
                 frame->nChannels != src.nChannels) {
                if(!frame)
                {
                   // printf("frame = null , create frame\n");
                }
                else
                {
                    printf("recreate frame  because mismatch , (frame->width %d , src.width %d) (frame->height %d , src.height %d) (frame->depth %d , src.depth %d) (frame->nChannels %d , src.nChannels %d)\n",frame->width,src.width,frame->height,src.height,frame->depth,src.depth,frame->nChannels,src.nChannels);
                }
                cvReleaseImage( &frame );
                frame = cvCreateImage( cvGetSize(&src), src.depth, channels );
            }
            cvCopy(&src, frame);

            return frame;
        }
    }
    return NULL;
}

void CvCaptureCAM_PVCAM::autoExposureControl(IplImage* image)
{
  
   printf("CvCaptureCAM_PVCAM::autoExposureControl\n");
   
}

double CvCaptureCAM_PVCAM::getProperty( int property_id ) const
{
   // printf("CvCaptureCAM_PVCAM::getProperty\n");
    switch(property_id) {

        case CV_CAP_PROP_CONVERT_RGB:
            break;

        case CV_CAP_PROP_TRIGGER:
            break;

        case CV_CAP_PROP_POS_MSEC:
            break;     //(double)frameID/fps;


        case CV_CAP_PROP_PVCAM_FRAME_WIDTH_MAX:
            return camera_detected_width;

        case CV_CAP_PROP_PVCAM_FRAME_HEIGHT_MAX:
            return camera_detected_height;

        case CV_CAP_PROP_PVCAM_SPEED_MIN:
            return min_speed;

        case CV_CAP_PROP_PVCAM_SPEED_MAX:
            return max_speed;

        case CV_CAP_PROP_PVCAM_SPEED:
            return current_speed;


        case CV_CAP_PROP_PVCAM_GAIN_MIN:
            return min_gain;

        case CV_CAP_PROP_PVCAM_GAIN_MAX:
            return max_gain;
  

        case CV_CAP_PROP_PVCAM_ROIX0:
            return xoffset;

        case CV_CAP_PROP_PVCAM_ROIY0:
            return yoffset;

        case CV_CAP_PROP_PVCAM_BINX:          
            return g_Region.sbin;
           
            
        case CV_CAP_PROP_PVCAM_BINY:          
            return g_Region.pbin;

        case CV_CAP_PROP_PVCAM_ROIX1: 
        case CV_CAP_PROP_FRAME_WIDTH:
            return width/g_Region.sbin;

        case CV_CAP_PROP_PVCAM_ROIY1:
        case CV_CAP_PROP_FRAME_HEIGHT:
            return height/g_Region.pbin;


        case CV_CAP_PROP_AUTO_EXPOSURE:
            return 0 ;  

        case CV_CAP_PROP_BRIGHTNESS:
             break;             

        case CV_CAP_PROP_EXPOSURE:
            if(exposureAvailable) {
                /* exposure time in seconds, like 1/100 s */
                return  exposure;     
            }
            break;

        case CV_CAP_PROP_FPS:
            if(fpsAvailable) {
                
            }
            break;

        case CV_CAP_PROP_GAIN:
            if(gainAvailable) {
                return current_gain; 
            }
            break;

        case CV_CAP_PROP_PVCAM_CLR_MODE:
            if(clear_mode_avail) {
                return clear_mode; 
            }
            break;

        case CV_CAP_PROP_PVCAM_CLR_CYCLES:
            if(clear_cycles_avail) {
                return clear_cycles; 
            }
            break;

        case CV_CAP_PROP_PVCAM_EXP_OUT_MODE:
            if(clear_expose_out_mode_avail) {
                return clear_expose_out_mode; 
            }
            break;

        case CV_CAP_PROP_TEMPERATURE:      
        case CV_CAP_PROP_PVCAM_TEMPERATURE:  
            if (PV_OK != pl_get_param(hcam, PARAM_TEMP, ATTR_CURRENT,
                (void *)&temperature)) {                
                   printf( "Can't get camera temperature\n"
                        );
                   break;
       
            }               
            return temperature; 
           
        case CV_CAP_PROP_FOURCC:
            {
              
                return MODE_Y16;
            }
            break;

        case CV_CAP_PROP_BUFFERSIZE:
           
            return 1; 
            break;
    }
    return -1.0;
}

bool CvCaptureCAM_PVCAM::setProperty( int property_id, double value )
{
   // printf("CvCaptureCAM_PVCAM::setProperty\n");
    unsigned int value_set;
    switch(property_id) {

        
        case CV_CAP_PROP_CONVERT_RGB:
            return false;

        case CV_CAP_PROP_TRIGGER:
            return false;

        case CV_CAP_PROP_PVCAM_FRAME_WIDTH_MAX:
            return false;

        case CV_CAP_PROP_PVCAM_FRAME_HEIGHT_MAX:
            return false;

        case CV_CAP_PROP_PVCAM_BINX:
             if(((uns16)value > 0) && ((uns16)value <= 16))
             {
                 
                 g_Region.sbin = (uns32)value; 
                 printf("Set CV_CAP_PROP_PVCAM_BINX  g_Region.sbin = %d\n",g_Region.sbin); 
                 return true;
             }
             else
             {
                 printf("Error , set CV_CAP_PROP_PVCAM_BINX  out of range\n");
                 return false;
             }
             break;

        case CV_CAP_PROP_PVCAM_BINY:
             if(((uns16)value > 0) && ((uns16)value <= 16))
             {
                 
                 g_Region.pbin = (uns32)value; 
                 printf("Set CV_CAP_PROP_PVCAM_BINY  g_Region.pbin = %d\n",g_Region.pbin); 
                 return true;
             }
             else
             {
                 printf("Error , set CV_CAP_PROP_PVCAM_BINY  out of range\n");
                 return false;
             }
             break;

        case CV_CAP_PROP_PVCAM_SPEED:
            if(((uns16)value >= min_speed) && ((uns16)value <= max_speed))
            {
               current_speed =  (uns16)value;
               printf("Set CV_CAP_PROP_PVCAM_SPEED value= %f\n",value);
              
               return true;
            }
            else
            {
               printf("Error , set CV_CAP_PROP_PVCAM_SPEED  out of range\n");
               return false;
            }
            break;

        case CV_CAP_PROP_AUTO_EXPOSURE:        
            return false;
            
        case CV_CAP_PROP_BRIGHTNESS:
            return false;
        

        case CV_CAP_PROP_EXPOSURE:
            if(exposureAvailable) {
                /* exposure time in seconds, like 1/100 s */
                //value *= 1e6; // -> from s to us
                exposure = value;
                printf("Set CV_CAP_PROP_EXPOSURE:  exposure = %f\n",exposure);

                break;
            } else return false;

        case CV_CAP_PROP_FPS:
            
            return false;

        case CV_CAP_PROP_GAIN:
            if(gainAvailable) {
            
              if(((uns16)value >= min_gain) && ((uns16)value <= max_gain))
              {
		 gain = value;
                 current_gain = (uns16)gain;
                 printf("Set CV_CAP_PROP_GAIN  gain = %f\n",gain);

                
                 return true;
              }
              else
              {
                 printf("Error , set CV_CAP_PROP_GAIN  out of range\n");
                 return false;
              }
                break;
            } else return false;

        case CV_CAP_PROP_FOURCC:
            {
                ArvPixelFormat newFormat = pixelFormat;
                switch((int)value) {
                    case MODE_GREY:
                    case MODE_Y800:
                        newFormat = ARV_PIXEL_FORMAT_MONO_8;
                        targetGrey = 128;
                        return false;
                    case MODE_Y12:
                        newFormat = ARV_PIXEL_FORMAT_MONO_12;
                        targetGrey = 2048;
                        return false;
                    case MODE_Y16:
                        newFormat = ARV_PIXEL_FORMAT_MONO_16;
                        targetGrey = 32768;
                        break;
                    case MODE_GRBG:
                        newFormat = ARV_PIXEL_FORMAT_BAYER_GR_8;
                        targetGrey = 128;
                        return false;
                }            
            }
            break;

        case CV_CAP_PROP_BUFFERSIZE:
            {             
                return false;
            }
        
        case CV_CAP_PROP_PVCAM_ROIX1:
        case CV_CAP_PROP_FRAME_WIDTH:
            value_set = (unsigned int)value;
            if((value_set >0) &&(value_set <= (camera_detected_width - xoffset)))
            {
		width = value_set;
                printf("Set CV_CAP_PROP_FRAME_WIDTH  width = %d\n",width);
                g_Region.s2 = xoffset + width -1;
                return true;
            }
            else
            {
                 printf("Error , set CV_CAP_PROP_FRAME_WIDTH  out of range\n");
                 return false;
            }
            break;
            
          
        case CV_CAP_PROP_PVCAM_ROIY1:
        case CV_CAP_PROP_FRAME_HEIGHT:
            value_set = (unsigned int)value;
            if((value_set >0) &&(value_set <= (camera_detected_height- yoffset)))
            {
		height = value_set;
                printf("Set CV_CAP_PROP_FRAME_HEIGHT  height = %d\n",height);
                g_Region.p2 = yoffset+ height -1;
                return true;
            }
            else
            {
                 printf("Error , set CV_CAP_PROP_FRAME_HEIGHT  out of range\n");
                 return false;
            }
            break;


        case CV_CAP_PROP_PVCAM_ROIX0:
            value_set = (unsigned int)value;
            if((value_set >0) &&(value_set < camera_detected_width))
            {
		xoffset = value_set;
                printf("Set CV_CAP_PROP_PVCAM_ROX0  xoffset = %d\n",xoffset);
                g_Region.s1 = xoffset;  
                return true;
            }
            else
            {
                 printf("Error , set CV_CAP_PROP_PVCAM_ROX0  out of range\n");
                 return false;
            }
            break;




        case CV_CAP_PROP_PVCAM_ROIY0:
            value_set = (unsigned int)value;
            if((value_set >0) &&(value_set < camera_detected_height))
            {
		yoffset = value_set;
                printf("Set CV_CAP_PROP_PVCAM_ROY0  yoffset = %d\n",yoffset);
                g_Region.p1 = yoffset;
                return true;
            }
            else
            {
                 printf("Error , set CV_CAP_PROP_PVCAM_ROY0  out of range\n");
                 return false;
            }
            break;

        case CV_CAP_PROP_PVCAM_CLR_MODE:
            if(clear_mode_avail) {
               value_set = (uns16)value;
               clear_mode = value_set;
               printf("Set CV_CAP_PROP_PVCAM_CLR_MODE  clear_mode = %d\n",clear_mode);
            }
            break;

        case CV_CAP_PROP_PVCAM_CLR_CYCLES:
            if(clear_cycles_avail) {
               value_set = (uns16)value;
               clear_cycles = value_set;
               printf("Set CV_CAP_PROP_PVCAM_CLR_CYCLES  clear_cycles = %d\n",clear_cycles);             
            }
            break;

        case CV_CAP_PROP_PVCAM_EXP_OUT_MODE:
            if(clear_expose_out_mode_avail) {

               value_set = (uns16)value;
               clear_expose_out_mode = value_set;
               printf("Set CV_CAP_PROP_PVCAM_EXP_OUT_MODE  clear_expose_out_mode = %d\n",clear_expose_out_mode); 
            }
            break;

        default:
            return false;
    }

    return true;
}

void CvCaptureCAM_PVCAM::stopCapture()
{
    printf("CvCaptureCAM_PVCAM::stopCapture\n");

    stop_capture();
}

bool CvCaptureCAM_PVCAM::startCapture()
{
    printf("CvCaptureCAM_PVCAM::startCapture ...\n");
   
    if(init_buffers() ) {
        printf("before calling start_capture  exposure %f\n",exposure);
        start_capture(exposure);
        return true;
    }
    return false;
}

CvCapture* cvCreateCameraCapture_PVCAM( int index )
{
    printf("cvCreateCameraCapture_PVCAM , index %d\n",index);
    CvCaptureCAM_PVCAM* capture = new CvCaptureCAM_PVCAM;

    if(capture->open(index)) {
        return capture;
    }

    delete capture;
    return NULL;
}
#endif
