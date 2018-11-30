#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc_c.h>
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>


using namespace cv;


long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}


int main(int argc, char** argv)
{
      unsigned char * img_buf;
      IplImage* frame;
      printf("Press ESC to exit\n");
     // cvNamedWindow( "First Example of PVAPI Integrated", CV_WINDOW_AUTOSIZE );
      CvCapture* capture = cvCreateCameraCapture( CV_CAP_PVCAM_PCIE );
     // assert( capture != NULL );
      if(capture == NULL)
      {
          printf("capture = null error return.\n");
          return 1;
      }

      double frame_max_width = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_FRAME_WIDTH_MAX);
      double frame_max_height = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_FRAME_HEIGHT_MAX);
      printf("frame_max_width = %f , frame_max_height = %f\n",frame_max_width, frame_max_height);
      double speed_min = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_SPEED_MIN);
      double speed_max = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_SPEED_MAX);
      printf("speed_min = %f , speed_max = %f\n",speed_min, speed_max);
      double gain_min = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_GAIN_MIN);
      double gain_max = cvGetCaptureProperty(capture,CV_CAP_PROP_PVCAM_GAIN_MAX);
      printf("gain_min = %f , gain_max = %f\n",gain_min, gain_max);

      cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_SPEED,speed_min);

      cvSetCaptureProperty(capture,CV_CAP_PROP_GAIN,gain_min);

      unsigned int counts =0;
      long long start_time = current_timestamp();
      for(int i=0;i<450;i++)
      {
            if( i == 50)
            {
               //printf("index 10, set exposure 15 ms\n");
               //cvSetCaptureProperty(capture,CAP_PROP_EXPOSURE,0.015); 
               //cvSetCaptureProperty(capture,CV_CAP_PROP_GAIN,2);


              /* cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIX0,500);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIY0,500); 
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIX1,500);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIY1,500); 
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_BINX,2);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_BINY,1); */
              // printf("cvSetCaptureProperty CV_CAP_PROP_PVCAM_CLR_MODE\n");
              // cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_CLR_MODE,2);
            
            }
            else if( i == 300)
            { 
               //cvSetCaptureProperty(capture,CV_CAP_PROP_GAIN,1);

              /* cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIX0,200);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIY0,200); 
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIX1,700);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_ROIY1,700); 
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_BINX,1);
               cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_BINY,2); */
               //cvSetCaptureProperty(capture,CV_CAP_PROP_PVCAM_CLR_CYCLES,1);
            }
            frame = cvQueryFrame(capture);
           // printf("frame  = 0x%x\n",(unsigned int)frame);
            if(frame)
            {
               int width = frame->width;
               int height = frame->height;
              // int depth = frame->depth;
               int imageSize = frame->imageSize;
               char * imageData = frame->imageData;
             
               //printf("width = %d , height = %d ,imageSize = %d\n",width, height, imageSize);
               //printf("depth = %d , imageSize = %d\n",depth, imageSize);
               //img_buf = (unsigned char * )imageData;
               //printf("0x%x , 0x%x , 0x%x , 0x%x , 0x%x\n",img_buf[0],img_buf[1],img_buf[2],img_buf[3],img_buf[4]);
               //counts += 1;
               cvShowImage( "Pvcam camera", frame);

            }
            else
            {
               printf("frame point is zero\n");
	       continue;
            }
           
            char c = cvWaitKey(10);    //(2500);
            if( c == 27) break;
         
      }
    long long end_time = current_timestamp();
    printf("start milliseconds: %lld\n", end_time);
    long long diff_time = end_time - start_time;
    printf("diff_time (ms): %lld\n", diff_time);
    if(diff_time > 1000)
    {
         long long fps = counts /(diff_time/1000);

         printf("fps: %d\n", fps);
      }
      
      cvReleaseCapture( &capture );
      cvDestroyWindow( "First Example of PVAPI Integrated" );
      printf("return\n");

 /*   unsigned int counts =0;

    VideoCapture  cap(0);            //cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
    { 
        printf("error , not opened !!\n");
        return -1;
    }

    //Mat frame;
    //uchar * data;
 
   // cap.set(CAP_PROP_EXPOSURE,15); 
   // cap.set(CAP_PROP_FPS,50); 

   // double auto_exp_value = cap.get(CAP_PROP_AUTO_EXPOSURE);
   // double exp_value = cap.get(CAP_PROP_EXPOSURE);
   // double fps_value = cap.get(CAP_PROP_FPS);

   // printf("auto_exp_value %f\n",auto_exp_value);
   // printf("exp_value %f\n",exp_value);
   // printf("fps_value %f\n",fps_value);

    long long start_time = current_timestamp();
    printf("start milliseconds: %lld\n", start_time);
    for(int i=0;i<10;i++)
    {
        Mat frame;
        uchar * data;
        cap >> frame; // get a new frame from camera

        data = frame.data;
        if(data)
        {
           counts += 1;
           //memset(data , 0x0f, 1920*1080*2);
           printf("count %d  , first 5 bytes is 0x%x,0x%x,0x%x,0x%x,0x%x\n", counts , data[0], data[1],data[2], data[3],data[4]);
        }
        else
        {
            printf("data is null , = 0x%x \n", data);
        }
       // usleep(10000);



       // if((i%10) == 0)
        //{
        // printf("\n\n Sleep 1 second \n\n\n");
        // sleep(1);
       // }

       // imshow("VideoCapture Basic Demo", frame);

       // char c = cvWaitKey(10);    //(2500);
       // if( c == 27) break;
           
    }
    printf("last frame counts %d\n",counts);
    long long end_time = current_timestamp();
    printf("start milliseconds: %lld\n", end_time);
    long long diff_time = end_time - start_time;
    printf("diff_time (ms): %lld\n", diff_time);
    if(diff_time > 1000)
    {
       long long fps = counts /(diff_time/1000);

       printf("fps: %d\n", fps);
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    printf("return \n");
    return 0;*/

}
