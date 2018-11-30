#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc_c.h>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>


using namespace cv;
using namespace std;


long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}


int main(int argc, char** argv)
{
      int choice = 0;
      printf("Select camera: 1: PCIE 2: USB 3:first detected\n");
      cin >>choice;
      printf("Camera selected choice = %d\n",choice);
    
      if((choice != 1) && (choice != 2) &&(choice != 3))
      {
         printf("Selected wrong number.  valid option is : 1: PCIE 2: USB 3:first detected\n");
      }

    
      if(choice != 3)
      {
      	unsigned char * img_buf;
      	IplImage* frame;
     
      	CvCapture* capture = NULL;
        char camera_name[64] = {0};
      
      	if(choice == 1)
      	{
          printf("Try open PCIE camera\n");
          strcpy(camera_name ,"PCIE camera demo");
          capture = cvCreateCameraCapture( CV_CAP_PVCAM_PCIE );
      	}
      	else if(choice == 2)
      	{
          printf("Try open USB camera\n");
          strcpy(camera_name ,"USB camera demo");
          capture = cvCreateCameraCapture( CV_CAP_PVCAM_USB );
      	}
  
      	if(capture == NULL)
      	{
          printf("Can not find camera error\n");
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

      	printf("Press ESC to exit \n");
      	for(int i=0;i<450;i++)
      	{
            
            frame = cvQueryFrame(capture);
            if(frame)
            {
               cvShowImage( camera_name, frame);

            }
            else
            {
               printf("frame point is zero\n");
	       continue;
            }
           
            char c = cvWaitKey(10);    
            if( c == 27) break;
         
      	}
   
      	cvReleaseCapture( &capture );
      	cvDestroyWindow( "First Example of PVAPI Integrated" );
      	printf("return\n");
      }
      else
      {
        printf("Press ESC to exit \n");
    	unsigned int counts =0;
    	VideoCapture  cap(0);   // open the default camera
    	if(!cap.isOpened())  // check if we succeeded
    	{ 
          printf("Error,Can not find camera !!\n");
          return -1;
        }

        for(int i=0;i<450;i++)
        {
          Mat frame;
          uchar * data;
          cap >> frame; // get a new frame from camera
          data = frame.data;     
          imshow("Default first detected camera demo", frame);
          char c = cvWaitKey(10);   
          if( c == 27) break;          
        }
        printf("return \n");
        return 0;
     }

}
