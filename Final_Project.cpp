/***************************************************************************************
* FILENAME: capture.cpp                                                                     
* OWNER: Sridhar Pavithrapu & Subhradeep Dutta.							       
* FILE DESCRIPTION: This file includes the function definitions of capture.cpp 
***************************************************************************************/

/* Headers section */
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <errno.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <syslog.h>

/* Namespace section */
using namespace cv;
using namespace std;

/* Macro's definition */
#define HRES 640
#define VRES 480
#define NSEC_PER_SEC (1000000000)
#define ERROR (-1)
#define OK (0)
#define MAX_OBJECTS (5)

/* Global Variable Declarations */
pthread_t FrameCapture_Thread;
pthread_t CentroidDetection_Thread;
pthread_t ObstacleMovement_Thread;
pthread_t Sequencer_Thread;
pthread_t video_Thread;
pthread_attr_t FrameCapture_sched_attr;
pthread_attr_t CentroidDetection_sched_attr;
pthread_attr_t ObstacleMovement_sched_attr;
pthread_attr_t main_sched_attr;
pthread_attr_t Sequencer_sched_attr;
pthread_attr_t video_sched_attr;
struct sched_param FrameCapture_param;
struct sched_param CentroidDetection_param;
struct sched_param ObstacleMovement_param;
struct sched_param main_param;
struct sched_param Sequencer_param;
struct sched_param video_param;
int trajectory_count = 0;
pthread_mutex_t rsrc_frameCapture, rsrc_Coordinates,rsrc_video;
static sem_t coordinates_sem, obstacle_sem;
/* Names of the created windows */
char centroid_window_name[] = "Centroid Detection";
char obstacle_window_name[] = "Obstacle Movement";
int dev=0;
IplImage* frame_original;
CvCapture* capture;
bool answer = false;
static int video_count = 0;
vector<Mat> various_images;
int coordinate_x, coordinate_y;
/* Maximum number of objects to be detected */
const int MAX_NUMBER_OF_OBJECTS = MAX_OBJECTS;
/* minimum and maximum object area */
const int MIN_AREA_OF_OBJECT = 60*60;
const int MAX_AREA_OF_OBJECT = VRES*HRES/1.5;
/* Frequency calculation variables for different services */
static struct timespec rtclk_frameCapture_start_time = {0, 0};
static struct timespec rtclk_frameCapture_stop_time = {0, 0};
static struct timespec rtclk_frameCapture_difference = {0,0};
static struct timespec rtclk_centroidDetection_start_time = {0, 0};
static struct timespec rtclk_centroidDetection_stop_time = {0, 0};
static struct timespec rtclk_centroidDetection_difference = {0,0};
static struct timespec rtclk_obstacleMovement_start_time = {0, 0};
static struct timespec rtclk_obstacleMovement_stop_time = {0, 0};
static struct timespec rtclk_obstacleMovement_difference = {0,0};
static struct timespec rtclk_videoOuput_start_time = {0, 0};
static struct timespec rtclk_videoOuput_stop_time = {0, 0};
static struct timespec rtclk_videoOuput_difference = {0,0};


#ifdef FREQUENCY_FRAMECAPTURE
int framecapture_count = 0;
#endif

#ifdef FREQUENCY_CENTROIDDETECTION
int centroiddetection_count = 0;
#endif

#ifdef FREQUENCY_OBSTACLEMOVEMENT
int obstaclemovement_count = 0;
#endif

#ifdef FREQUENCY_VIDEOCAPTURE
int videocapture_count = 0;
#endif

/********************************************************************
Function definition for calculating the time difference
********************************************************************/
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  /* Calculating the time difference */
  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  return(OK);
}

/********************************************************************
Function definition for printing the scheduling policy
********************************************************************/
void print_scheduler(void)
{
	int schedType;
	schedType = sched_getscheduler(getpid());
	switch(schedType)
	{
		case SCHED_FIFO:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_FIFO\n");
		break;

		case SCHED_OTHER:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_OTHER\n");
		break;

		case SCHED_RR:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_RR\n");
		break;

		default:
			syslog(LOG_INFO, "%s", "Pthread Policy is UNKNOWN\n");
		break;
	}
}

/********************************************************************
Function definition for frame capture thread
********************************************************************/
void *FrameCapture(void * unused)
{

	syslog(LOG_INFO, "%s", "Entered Frame Capture\n");
	
	while(1)
    {	
		#ifdef FREQUENCY_FRAMECAPTURE
		/*When count of captured frames is zero then start the timer*/
		if(framecapture_count == 0)
		{
			clock_gettime(CLOCK_REALTIME, &rtclk_frameCapture_start_time);
			syslog(LOG_INFO, "FrameCapture thread count %d starts time :: sec= %ld :: nsec= %ld \n",framecapture_count, rtclk_frameCapture_start_time.tv_sec, rtclk_frameCapture_start_time.tv_nsec);
		}
		#endif
		
		/* Capturing the input frame from the connected camera device */
		syslog(LOG_INFO, "%s", "FrameCapture thread grabbing frame resource\n");
		/*Mutex lock prior to frame capture*/
		pthread_mutex_lock(&rsrc_frameCapture);
		frame_original=cvQueryFrame(capture);
		/*Mutex unlocked after frame capture complete*/
		pthread_mutex_unlock(&rsrc_frameCapture);

		#ifdef FREQUENCY_FRAMECAPTURE
		framecapture_count++;
		/*Stop the timer once 500 frames have been captured*/
		if(framecapture_count == 500)
		{		
			clock_gettime(CLOCK_REALTIME, &rtclk_frameCapture_stop_time);
			syslog(LOG_INFO, "FrameCapture thread count %d stop time :: sec= %ld :: nsec= %ld \n",framecapture_count, rtclk_frameCapture_stop_time.tv_sec, rtclk_frameCapture_stop_time.tv_nsec);
			delta_t(&rtclk_frameCapture_stop_time, &rtclk_frameCapture_start_time, &rtclk_frameCapture_difference);
			syslog(LOG_INFO, "FrameCapture thread difference time :: sec= %ld :: nsec= %ld \n", rtclk_frameCapture_difference.tv_sec, rtclk_frameCapture_difference.tv_nsec);
			exit(0);		
		}
		#endif
	}
}


/********************************************************************
Function definition for centroid detection thread
********************************************************************/
void *CentroidDetection(void * unused) 
{
	syslog(LOG_INFO, "%s", "Entered Centroid Detection\n");
	Mat hsv_image, final_image, erode_structureElement, dilate_structureElement, temp_image;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
	Moments moment;
	stringstream x_coordinate,y_coordinate;
	stringstream final_coordinates;
    double area;
	/* Values for green color object detection */     
    /* Set Hue */
    int lowH = 34;       
    int highH = 80;
    /* Set Saturation */
    int lowS = 50;      
    int highS = 220;
    /* Set Value */
    int lowV = 50;      
    int highV = 200;
	
	while(1)
    {	
		sem_wait(&coordinates_sem);	
		
		#ifdef FREQUENCY_CENTROIDDETECTION
		/*When centroid detection count is zero start the timer*/
		if(centroiddetection_count == 0)
		{
			clock_gettime(CLOCK_REALTIME, &rtclk_centroidDetection_start_time);
			syslog(LOG_INFO, "Centroid detection thread count %d starts time :: sec= %ld :: nsec= %ld \n",centroiddetection_count, rtclk_centroidDetection_start_time.tv_sec, rtclk_centroidDetection_start_time.tv_nsec);
		}
		#endif

		
		syslog(LOG_INFO, "%s", "Centroid Detection thread grabbing frame resource\n");
		/*Mutex lock prior to creating a copy of the input frame*/
		pthread_mutex_lock(&rsrc_frameCapture);	
		/* Creating the Mat object 'mat_frame' to create a copy using the copy constructor */		
		Mat mat_frame(frame_original);
		/*Mutex unlocked after duplicating the captured frame*/
		pthread_mutex_unlock(&rsrc_frameCapture);
		/* Smoothing the image to remove noise */
		medianBlur(mat_frame, mat_frame, 3);
		/* Changing the color space from RGB to HSV */
		/* In order to apply erosion dilation HSV color space is a requirement*/
		cvtColor(mat_frame, hsv_image, CV_BGR2HSV);
		/* Change the range to detect green color object */
		/*Defines what we are looking for in the input image, in this case green colored objects that satisfy the below condition*/
        inRange(hsv_image, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), final_image);
		/* Applying erode and dilation filters to the above modified image */
		erode_structureElement = getStructuringElement( MORPH_RECT,Size(3,3));
        dilate_structureElement = getStructuringElement( MORPH_RECT,Size(8,8));
        erode(final_image,final_image,erode_structureElement);
        erode(final_image,final_image,erode_structureElement);
        dilate(final_image,final_image,dilate_structureElement);
        dilate(final_image,final_image,dilate_structureElement);
		final_image.copyTo(temp_image);
		/* Find contours of filtered image using openCV findContours function */
        findContours(temp_image,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
		/* Condition for checking the objects found through contours */
		/*Size parameter of hierarchy defines the number of contours that were found*/
        if (hierarchy.size() > 0) 
        {
            int numObjects = hierarchy.size();
            /* If the number of objects greater than MAX_NUM_OBJECTS we have a noisy filter */
            if(numObjects<MAX_NUMBER_OF_OBJECTS){
                for (int index = 0; index >= 0; index = hierarchy[index][0])//Iterating through each of the contours
				{
					moment = moments((cv::Mat)contours[index]);
					/*Area of the contour under consideration*/
					area = moment.m00;
					/* Check for removing stray circles which do not meet a specific threshold */
					if(area>MIN_AREA_OF_OBJECT)
					{
						drawContours(mat_frame,contours,-1,CV_RGB(255,0,0),3);
						/* Mutex lock prior to determining the x and y coordinates */
						pthread_mutex_lock(&rsrc_Coordinates);
						coordinate_x = (moment.m10/area);
						coordinate_y = (moment.m01/area);
						/* Set answer to true indicating there is a valid axes value*/
						answer = true;
						pthread_mutex_unlock(&rsrc_Coordinates);
						(x_coordinate << coordinate_x);
						(y_coordinate << coordinate_y);
						final_coordinates << "(" << x_coordinate.str() << "," << y_coordinate.str() << ")";
						circle(mat_frame,Point((moment.m10/area),(moment.m01/area)),5,CV_RGB(255,0,0));
						putText(mat_frame,final_coordinates.str(),Point((moment.m10/area),(moment.m01/area)),5,1,CV_RGB(255,0,0));
						final_coordinates.str(std::string());
						x_coordinate.str(std::string());
						y_coordinate.str(std::string());
                    }    
                }
            }
        } 
		
		/* Storing the images in the vector for video writer thread */
		pthread_mutex_lock(&rsrc_video);
		various_images.push_back(mat_frame);
		/*Keep track if a valid image frame is available */
		video_count++;
		pthread_mutex_unlock(&rsrc_video);
        
		imshow( centroid_window_name, mat_frame);
        char q = cvWaitKey(1);
		
		#ifdef FREQUENCY_CENTROIDDETECTION
		centroiddetection_count++;
		/*When centroid detection count is 500 stop the timer*/
		if(centroiddetection_count == 500)
		{		
			clock_gettime(CLOCK_REALTIME, &rtclk_centroidDetection_stop_time);
			syslog(LOG_INFO, "Centroid detection thread count %d stop time :: sec= %ld :: nsec= %ld \n",centroiddetection_count, rtclk_centroidDetection_stop_time.tv_sec, rtclk_centroidDetection_stop_time.tv_nsec);
			delta_t(&rtclk_centroidDetection_stop_time, &rtclk_centroidDetection_start_time, &rtclk_centroidDetection_difference);
			syslog(LOG_INFO, "Centroid detection thread difference time :: sec= %ld :: nsec= %ld \n", rtclk_centroidDetection_difference.tv_sec, rtclk_centroidDetection_difference.tv_nsec);
			exit(0);		
		}
		#endif
	}
}


/********************************************************************
Function definition for obstacle movement thread
********************************************************************/
void *ObstacleMovement(void * unused) 
{
	syslog(LOG_INFO, "%s", "Entered Obstacle Movement\n");
	int obstacle_yaxes = 480;
	int obstacle_xaxes;
	stringstream x_coordinate,y_coordinate, final_coordinates;
	Mat drawing=Mat::zeros( 480, 1281, CV_8UC3 );
	Point2i start_point(0,0),end_point(0,0);
    Mat obstacle_window,trajectory_window;
    obstacle_window = drawing(Rect(0,0,640,480));
    trajectory_window = drawing(Rect(641,0,640,480));
    line(drawing,Point(641,0),Point(641,480),CV_RGB(255,0,0),2);
	int trajectory_count =0;
	bool temp=false;
	
	while(1)
	{
		sem_wait(&obstacle_sem);
		
		#ifdef FREQUENCY_OBSTACLEMOVEMENT
		if(obstaclemovement_count == 0)
		{
			clock_gettime(CLOCK_REALTIME, &rtclk_obstacleMovement_start_time);
			syslog(LOG_INFO, "Obstacle Movement thread count %d starts time :: sec= %ld :: nsec= %ld \n",obstaclemovement_count, rtclk_obstacleMovement_start_time.tv_sec, rtclk_obstacleMovement_start_time.tv_nsec);
		}
		#endif
		
		syslog(LOG_DEBUG, "%s", "Obstacle thread grabbing frame resource\n");
		pthread_mutex_lock(&rsrc_Coordinates);
		obstacle_xaxes = coordinate_x;
		obstacle_yaxes = coordinate_y;
		temp = answer;
		pthread_mutex_unlock(&rsrc_Coordinates);
		/* Copying the coordinate of the object for trajectory */
		if(trajectory_count == 0 && temp == true)
		/*Set start position for tracing the points on the screen*/
		{		
			start_point.x = obstacle_xaxes;		
			start_point.y = obstacle_yaxes;
			trajectory_count++;
		}
		else if(trajectory_count > 0 && temp == true)
		/*Keep adding new points to continue tracing the path*/
		{	
			end_point.x = obstacle_xaxes;		
			end_point.y = obstacle_yaxes;
			line(trajectory_window,start_point,end_point,CV_RGB(255,0,0),4,8); 
			start_point.x = end_point.x;
			start_point.y = end_point.y;	
		}
		else
		{
			
		}
		/* Check for corner case conditions */
		if(obstacle_xaxes < 20)
		{
            obstacle_xaxes = 20;
		}
		else if(obstacle_xaxes > 620)
		{
			obstacle_xaxes = 620;
		}
		else
		{
			
		}
		
		line(obstacle_window,Point(obstacle_xaxes-20,480),Point(obstacle_xaxes+20,480),CV_RGB(255,0,0),12,8);  
		(x_coordinate << obstacle_xaxes);
		(y_coordinate << 480);
		final_coordinates << "(" << x_coordinate.str() << "," << y_coordinate.str() << ")";
		putText(obstacle_window,final_coordinates.str(),Point((obstacle_xaxes),(480-10)),5,1,CV_RGB(255,0,0));
		imshow(obstacle_window_name,drawing);
		char q = waitKey(2);
		if(q == 'q')
		{
			/* Clearing the trajectory window to draw new trajectory plot */
			trajectory_window.setTo(Scalar(0,0,0));
		}
		/* Clearing the obstacle window to draw new line */
		obstacle_window.setTo(Scalar(0,0,0));
		final_coordinates.str(std::string());
		x_coordinate.str(std::string());
		y_coordinate.str(std::string());
		
		#ifdef FREQUENCY_OBSTACLEMOVEMENT
		obstaclemovement_count++;
		if(obstaclemovement_count == 500)
		{		
			clock_gettime(CLOCK_REALTIME, &rtclk_obstacleMovement_stop_time);
			syslog(LOG_INFO, "Obstacle movement thread count %d stop time :: sec= %ld :: nsec= %ld \n",obstaclemovement_count, rtclk_obstacleMovement_stop_time.tv_sec, rtclk_obstacleMovement_stop_time.tv_nsec);
			delta_t(&rtclk_obstacleMovement_stop_time, &rtclk_obstacleMovement_start_time, &rtclk_obstacleMovement_difference);
			syslog(LOG_INFO, "Obstacle movement thread difference time :: sec= %ld :: nsec= %ld \n", rtclk_obstacleMovement_difference.tv_sec, rtclk_obstacleMovement_difference.tv_nsec);
			exit(0);		
		}
		#endif
		
	}
}

/********************************************************************
Function definition for video output thread
********************************************************************/
void *Video_Output(void * unused) 
{
	syslog(LOG_INFO, "%s", "Entered Video writer thread\n");
	/* Opening a video writer */
	cv::VideoWriter output_cap("/home/subhradeep/video/video_output/sri.avi",
               CV_FOURCC('M','J','P','G'), 1, cv::Size ( 640,480), true);
	if (!output_cap.isOpened())
	{
        	std::cout << "!!! Output video could not be opened" << std::endl;
        	pthread_exit(NULL);
	}
	Mat retrieve_element; 
	
	while(1)
	{
		
		#ifdef FREQUENCY_VIDEOCAPTURE
		if(videocapture_count == 0)
		{
			clock_gettime(CLOCK_REALTIME, &rtclk_videoOuput_start_time);
			syslog(LOG_INFO, "Video output thread count %d starts time :: sec= %ld :: nsec= %ld \n",videocapture_count, rtclk_videoOuput_start_time.tv_sec, rtclk_videoOuput_start_time.tv_nsec);
		}
		#endif
		
		/* Copying the frames to video writer */
		pthread_mutex_lock(&rsrc_video);
		if(video_count != 0)
		{
			various_images[video_count].copyTo(retrieve_element);
			output_cap.write(retrieve_element);
			/*Decrement video_count to indicate that no valid frame exists*/
			video_count--;
		}
		pthread_mutex_unlock(&rsrc_video);
		
		#ifdef FREQUENCY_VIDEOCAPTURE
		videocapture_count++;
		if(videocapture_count == 500)
		{		
			clock_gettime(CLOCK_REALTIME, &rtclk_videoOuput_stop_time);
			syslog(LOG_INFO, "Video output thread count %d stop time :: sec= %ld :: nsec= %ld \n",videocapture_count, rtclk_obstacleMovement_stop_time.tv_sec, rtclk_obstacleMovement_stop_time.tv_nsec);
			delta_t(&rtclk_videoOuput_stop_time, &rtclk_videoOuput_start_time, &rtclk_videoOuput_difference);
			syslog(LOG_INFO, "Video output thread difference time :: sec= %ld :: nsec= %ld \n", rtclk_videoOuput_difference.tv_sec, rtclk_videoOuput_difference.tv_nsec);
			exit(0);		
		}
		#endif
	}
	output_cap.release();
}

/********************************************************************
Function definition for sequencer thread
********************************************************************/
void *Sequencer(void * unused)
{	
	while(1)
	{
		/* Sequence of threads */
		usleep(125000);
		sem_post(&coordinates_sem);
		usleep(75000);
		sem_post(&obstacle_sem);
		usleep(50000);
		sem_post(&coordinates_sem);
		usleep(125000);
		sem_post(&coordinates_sem);
		usleep(25000);
		sem_post(&obstacle_sem);
		usleep(100000);
		sem_post(&coordinates_sem);
		usleep(100000);
		sem_post(&obstacle_sem);
		usleep(25000);
		sem_post(&coordinates_sem);
		usleep(125000);
		sem_post(&coordinates_sem);
		usleep(50000);
		sem_post(&obstacle_sem);
		usleep(75000);
		sem_post(&coordinates_sem);
		usleep(125000);
		sem_post(&coordinates_sem);
		sem_post(&obstacle_sem);
	}	
}

/********************************************************************
Function definition for main
********************************************************************/
int main (int argc, char *argv[])
{
	int rc , scope, rt_max_prio, rt_min_prio;
	cpu_set_t cpu_1,cpu_2,cpu_3;
	
	/* Configuring syslog */
	openlog("RTES_Final_Project",LOG_PID,LOG_LOCAL1 );
	
	/* Check for input argument during execution */
    if(argc > 1)
    {
        sscanf(argv[1], "%d", &dev);
		syslog(LOG_INFO, "using %s\n",argv[1]);
    }
    else if(argc == 1)
	{
        syslog(LOG_INFO, "%s", "using default\n");
	}
    else
    {
		syslog(LOG_INFO, "%s", "usage: capture [dev]\n");
        exit(-1);
    }
	
	/* Creating a window with AUTO_SIZE option for displaying the output */
    namedWindow( obstacle_window_name, CV_WINDOW_AUTOSIZE );
    namedWindow( centroid_window_name, CV_WINDOW_AUTOSIZE );

	/* Start capturing the frames from the input camera device */
    capture = (CvCapture *)cvCreateCameraCapture(dev);
	
	/* Setting the resolution using the cvSetCaptureProperty interface */
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);
	
	/* Printing the scheduler policy */
	syslog(LOG_DEBUG, "%s", "Before adjustments to scheduling policy:\n");
	print_scheduler();
	
	/* Initialization of CPU set for different threads*/
    CPU_ZERO(&cpu_1);
    CPU_SET(0, &cpu_1);
    CPU_ZERO(&cpu_2);
    CPU_SET(1, &cpu_2);
    CPU_ZERO(&cpu_3);
    CPU_SET(2, &cpu_3);

	/* Initialization of semaphores */

	
	if (sem_init(&coordinates_sem, 0, 1) == -1)
    {
		syslog(LOG_ERR, "sem_init for coordinates_sem: failed: %s\n", strerror(errno)); 
	}

	if (sem_init(&obstacle_sem, 0, 1) == -1)
    { 
		syslog(LOG_ERR, "sem_init for obstacle_sem: failed: %s\n", strerror(errno)); 
	}


	/* Set default protocol for mutex */
  	pthread_mutex_init(&rsrc_frameCapture, NULL);
   	pthread_mutex_init(&rsrc_Coordinates, NULL);
	pthread_mutex_init(&rsrc_video, NULL);
	
	/* Assigning the attributes for the threads */
	pthread_attr_init (&main_sched_attr);
	pthread_attr_init (&FrameCapture_sched_attr);
	pthread_attr_init (&CentroidDetection_sched_attr);
	pthread_attr_init (&ObstacleMovement_sched_attr);
	pthread_attr_init (&Sequencer_sched_attr);
	pthread_attr_init (&video_sched_attr);
	
	pthread_attr_setinheritsched (&main_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&main_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&main_sched_attr, sizeof(cpu_1), &cpu_1);
	
	pthread_attr_setinheritsched (&FrameCapture_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&FrameCapture_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&FrameCapture_sched_attr, sizeof(cpu_2), &cpu_2);
	
	pthread_attr_setinheritsched (&CentroidDetection_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&CentroidDetection_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&CentroidDetection_sched_attr, sizeof(cpu_1), &cpu_1);
	
	pthread_attr_setinheritsched (&ObstacleMovement_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&ObstacleMovement_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&ObstacleMovement_sched_attr, sizeof(cpu_1), &cpu_1);

	pthread_attr_setinheritsched (&Sequencer_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&Sequencer_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&Sequencer_sched_attr, sizeof(cpu_1), &cpu_1);

	pthread_attr_setinheritsched (&video_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&video_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&video_sched_attr, sizeof(cpu_3), &cpu_3);
	
	
	rt_max_prio = sched_get_priority_max (SCHED_FIFO);
	rt_min_prio = sched_get_priority_min (SCHED_FIFO);
	
	/* Assigning priority to threads */
	rc = sched_getparam (getpid(), &main_param);
	main_param.sched_priority = rt_max_prio;
	FrameCapture_param.sched_priority = rt_max_prio-20;
	CentroidDetection_param.sched_priority = rt_max_prio-30;
	ObstacleMovement_param.sched_priority = rt_max_prio-40;
	Sequencer_param.sched_priority = rt_max_prio-10;
	video_param.sched_priority = rt_max_prio-50;
	
	/* Assigning main thread with scheduling policy FIFO */ 
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
	if (rc)
	{
		syslog(LOG_ERR, "sched_setscheduler rc is %d\n",rc);
		perror(NULL); exit(-1);
	}
	
	/* Printing the scheduler policy after changing it to FIFO */
	syslog(LOG_DEBUG, "%s", "After adjustments to scheduling policy:\n");
	print_scheduler();
	syslog(LOG_INFO, "min prio = %d, max prio = %d\n", rt_min_prio, rt_max_prio);
	
	pthread_attr_setschedparam (&main_sched_attr, &main_param);	
	pthread_attr_setschedparam (&FrameCapture_sched_attr, &FrameCapture_param);
	pthread_attr_setschedparam (&CentroidDetection_sched_attr, &CentroidDetection_param);
	pthread_attr_setschedparam (&ObstacleMovement_sched_attr, &ObstacleMovement_param);
	pthread_attr_setschedparam (&Sequencer_sched_attr, &Sequencer_param);
	pthread_attr_setschedparam (&video_sched_attr, &video_param);
	
	/* Creation of threads */
	
	rc = pthread_create (&Sequencer_Thread , &Sequencer_sched_attr , Sequencer , NULL );
	if (rc)
	{
		syslog(LOG_ERR, "ERROR; pthread_create() rc is %d\n", rc);
		perror(NULL); 
		exit(-1);
	}
	
	rc = pthread_create (&FrameCapture_Thread , &FrameCapture_sched_attr , FrameCapture , NULL );
	if (rc)
	{
		syslog(LOG_ERR, "ERROR; pthread_create() rc is %d\n", rc);
		perror(NULL); 
		exit(-1);
	}
	
	rc = pthread_create (&CentroidDetection_Thread , &CentroidDetection_sched_attr , CentroidDetection , NULL );
	if (rc)
	{
		syslog(LOG_ERR, "ERROR; pthread_create() rc is %d\n", rc);
		perror(NULL); 
		exit(-1);
	}
	
	rc = pthread_create (&ObstacleMovement_Thread , &ObstacleMovement_sched_attr , ObstacleMovement , NULL );
	if (rc)
	{
		syslog(LOG_ERR, "ERROR; pthread_create() rc is %d\n", rc);
		perror(NULL); 
		exit(-1);
	}

	rc = pthread_create (&video_Thread , &video_sched_attr , Video_Output , NULL );
	if (rc)
	{
		syslog(LOG_ERR, "ERROR; pthread_create() rc is %d\n", rc);
		perror(NULL); 
		exit(-1);
	}
	

    /* Suspending main until all the threads complete its execution */
	pthread_join ( FrameCapture_Thread , NULL );
	pthread_join ( CentroidDetection_Thread , NULL );
	pthread_join ( ObstacleMovement_Thread , NULL );
	pthread_join ( Sequencer_Thread , NULL );

	/* Releasing the resources used */ 
	if(pthread_attr_destroy(&FrameCapture_sched_attr) != 0)
		perror("attr destroy");
	
	if(pthread_attr_destroy(&CentroidDetection_sched_attr) != 0)
		perror("attr destroy");
	
	if(pthread_attr_destroy(&ObstacleMovement_sched_attr) != 0)
		perror("attr destroy");
	
	
	/* Stop capturing and free the resources */
    cvReleaseCapture(&capture);
	
	
	sem_destroy(&coordinates_sem);
	sem_destroy(&obstacle_sem);

	if(pthread_mutex_destroy(&rsrc_frameCapture) != 0)
		perror("mutex A destroy");

	if(pthread_mutex_destroy(&rsrc_video) != 0)
		perror("mutex A destroy");

   	if(pthread_mutex_destroy(&rsrc_Coordinates) != 0)
		perror("mutex B destroy");

	syslog(LOG_INFO, "%s","TEST COMPLETE");
	closelog();
}