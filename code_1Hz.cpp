/**
 * @file code_1Hz.cpp
 * 
 * @author Madhukar Arora 
 * @brief  source file to run image aquisition at 1Hz.
 * Refrences : http://opencv-tutorials-hub.blogspot.com/2016/04/how-to-put-text-on-image-in-OpenCV.html
 *             https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html
 *             https://stackoverflow.com/questions/7237144/compressing-images-on-opencv-imwrite-how-to-explicitly-set-the-compression-fa
 *             https://stackoverflow.com/questions/12233710/how-do-i-use-the-ostringstream-properly-in-c
 *             https://stackoverflow.com/questions/3596310/c-how-to-use-the-function-uname
 * 
 */

#define _GNU_SOURCE
/*
includes
*/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <sys/param.h>
#include <pthread.h>
#include <semaphore.h>
#include <syslog.h>
#include <errno.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mqueue.h>
#include <stdbool.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/utsname.h>
using namespace cv;
using namespace std;


/*
Macros
*/
#define USEC_PER_MSEC (1000)
#define MSEC (1000000)
#define NSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)
#define NUM_THREADS (5)
#define HRES 640
#define VRES 480



/**
 * Global Variables 
 */

int abortTest=FALSE, abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE;

sem_t semS1, semS2, semS3, semS4;
sem_t conv_ppm, ppm_complete,time_print,stamp1;
struct timeval start_time_val;

int captured_frames = 0;
struct utsname unameData;

//frames 
VideoCapture cap(0);
 
Mat ppm_window, jpg_window;
struct tm* ptr;

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;



static struct timespec capture_start = {0,0};
static struct timespec capture_stop  = {0,0};

static struct timespec save_ppm_start = {0,0};
static struct timespec save_ppm_stop = {0,0};

static struct timespec save_jpg_start = {0,0};
static struct timespec save_jpg_stop = {0,0};


double execution_time; 

static struct timespec code_start = {0,0};
static struct timespec code_end   = {0,0};
static struct timespec execution  = {0,0};
static struct timespec current_time = {0,0};

double start_time[4] = {0,0,0,0};
double end_time[4]   = {0,0,0,0};
double jitter[4]     = {0,0,0,0};
double avg_jitter[4] = {0,0,0,0};
double wcet[4]       = {0,0,0,0};
double avg_diff[4]   = {0,0,0,0};
uint32_t fcount[4]   = {0,0,0,0};

double total_jitter[4] = {0,0,0,0};
double run_time[4]   = {0,0,0,0};

uint8_t capture_flag = TRUE;




/*
Function to calculate the number of milliseconds
*/
double calc_ms(void)
{
	struct timespec frame = {0,0};
	clock_gettime(CLOCK_REALTIME, &frame);
	return ((frame.tv_sec*1000)+frame.tv_nsec/MSEC);
}



/*
Service to capture the frame. Uses OpenCV libraries
*/
void *capture_frame(void *threadp)
{
    while(!abortS1)
    {
        sem_wait(&semS1);
        //insert time functionality
        start_time[1] = calc_ms();
        clock_gettime(CLOCK_REALTIME,&capture_start);
        
        cap >> ppm_window;
        
        clock_gettime(CLOCK_REALTIME,&capture_stop);
        execution_time = (((capture_stop.tv_sec - capture_start.tv_sec) * 1000000000) + (capture_stop.tv_nsec - capture_start.tv_nsec));
        //printf("\n\rTime taken to capture frame : %f ns",execution_time);
        syslog(LOG_INFO,"\n\rTime taken to capture frame : %f ns",execution_time);
        end_time[1] = calc_ms();
        run_time[1] = end_time[1] - start_time[1];
        
        if(run_time[1] > wcet[1])
        {
            wcet[1] = run_time[1];
        }
        if(fcount[1] == 0)
        {
            avg_diff[1] = run_time[1];
        }  
        else if(fcount[1] > 0)
        {
            jitter[1] = run_time[1] - avg_diff[1];
            avg_diff[1] = (avg_diff[1] * (fcount[1]-1) + run_time[1])/(fcount[1]);
            total_jitter[1] += jitter[1];   
        }

        fcount[1]++;
    }
    //printf("\n\rWCET for Capture Thread : %f ms",wcet[1]);
    syslog(LOG_INFO,"\n\rWCET for Capture Thread : %f ms",wcet[1]);

    //printf("\n\rJitter for Capture Thread : %f ms",avg_jitter[1]);
    syslog(LOG_INFO,"\n\rJitter for Capture Thread : %f ms",total_jitter[1]);

    pthread_exit(NULL);

} 

/*
Service to save the PPM format image. Timestamps and System information is
added in this service thread.
*/
void *save_ppm(void *threadp)
{
    
    time_t t;
    //char filename[40];

    ostringstream name;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PXM_BINARY);
	compression_params.push_back(95);
 	uname(&unameData);
    while(!abortS2)
    {
        sem_wait(&semS2);
        sem_wait(&conv_ppm);
        
        
        clock_gettime(CLOCK_REALTIME,&save_ppm_start);
        // sprintf(filename,"PPM framenumber%d.ppm",fcount[1]);
        start_time[2] = calc_ms();
        name.str("Frame");
        name<<"Frame"<<fcount[2]<<".ppm"; 

        //timestamping each image before saving
        //https://www.geeksforgeeks.org/time-h-header-file-in-c-with-examples/
        time(&t);
        ptr = localtime(&t);

        char* timestamp = asctime(ptr);
        //print string to image
        putText(ppm_window,timestamp,Point(10,20),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        putText(ppm_window,unameData.sysname,Point(100,140),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        putText(ppm_window,unameData.nodename,Point(100,180),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        putText(ppm_window,unameData.release,Point(100,220),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        putText(ppm_window,unameData.version,Point(100,260),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        putText(ppm_window,unameData.machine,Point(100,300),FONT_HERSHEY_DUPLEX,0.8,Scalar(0,255,255),3);
        
        
        imwrite(name.str(),ppm_window,compression_params);

        clock_gettime(CLOCK_REALTIME,&save_ppm_stop);

        execution_time = (((save_ppm_stop.tv_sec - save_ppm_start.tv_sec) * 1000000000) + (save_ppm_stop.tv_nsec - save_ppm_start.tv_nsec));

        //printf("\n\rTime taken to save ppm form : %f ns",execution_time);
        syslog(LOG_INFO,"\n\rTime taken to save ppm form : %f ns",execution_time);        
        name.str(" ");
        
        
        end_time[2] = calc_ms();
        run_time[2] = end_time[2] - start_time[2];
       
        if(run_time[2] > wcet[2])
        {
            wcet[2] = run_time[2];
        }
        if(fcount[2] == 0)
        {
            avg_diff[2] = run_time[2];
        }  
        else if(fcount[2] > 0)
        {
            jitter[2] = run_time[2] - avg_diff[2];
            avg_diff[2] = (avg_diff[2] * (fcount[2]-1) + run_time[2])/(fcount[2]);
            total_jitter[2] += jitter[2];   
        }

        fcount[2]++;

        sem_post(&ppm_complete);
        sem_post(&stamp1);
        sem_post(&conv_ppm);    

    } 
    //jitter calculation
    avg_jitter[2] = total_jitter[2] / 2000;
    //printf("\n\rWCET for Save PPM Thread : %f ms",wcet[2]);
    syslog(LOG_INFO,"\n\rWCET for Save PPM Thread : %f ms",wcet[2]);

    //printf("\n\rJitter for  Save PPM : %f ms",avg_jitter[2]);
    syslog(LOG_INFO,"\n\rJitter for Save PPM: %f ms",total_jitter[2]);

    pthread_exit(NULL);
}




/*
Best effort service, function is to read the stored ppm frames 
and convert into compressed JPG files.
*/
void *save_jpg(void *threadp)
{
    ostringstream name;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(95);
 	//char filename[40];
    while(!abortS3)
    {
        sem_wait(&semS3);
        sem_wait(&ppm_complete);
        
        clock_gettime(CLOCK_REALTIME,&save_jpg_start);
        start_time[3] = calc_ms();
        name.str("Frame");
        name<<"Frame"<<fcount[3]<<".ppm"; 

        //sprintf(filename,"./PPM/framenumber%d.ppm",fcount[2]);
        jpg_window = imread(name.str(),CV_LOAD_IMAGE_COLOR);
        name.str("");

        name<<"Frame"<<fcount[3]<<".jpg";
        //sprintf(filename,"./JPG/framenumber%d.jpg",fcount[2]);
        imwrite(name.str(),jpg_window,compression_params);
        name.str(" ");


        clock_gettime(CLOCK_REALTIME,&save_jpg_stop);
        execution_time = (((save_jpg_stop.tv_sec - save_jpg_start.tv_sec) * 1000000000) + (save_jpg_stop.tv_nsec - save_jpg_start.tv_nsec));
        printf("\n\rTime taken to save ppm form : %f ns",execution_time);
        syslog(LOG_INFO,"\n\rTime taken to save jpg form : %f ns",execution_time);
       
        //jitter calculations
        end_time[3] = calc_ms();
        run_time[3] = end_time[3] - start_time[3];
        
        if(run_time[3] > wcet[3])
        {
            wcet[3] = run_time[3];
        }
        if(fcount[3] == 0)
        {
            avg_diff[3] = run_time[3];
        }  
        else if(fcount[3] > 0)
        {
            jitter[3] = run_time[3] - avg_diff[3];
            avg_diff[3] = (avg_diff[3] * (fcount[3]-1) + run_time[3])/(fcount[3]);
            total_jitter[3] += jitter[3];   
        }
        fcount[3]++;
        

    }
    avg_jitter[3] = total_jitter[3] / 2000;
    //printf("\n\rWCET for Save JPG Thread : %f ms",wcet[3]);
    syslog(LOG_INFO,"\n\rWCET for Save JPG Thread : %f ms",wcet[3]);

    //printf("\n\rJitter for  Save JPG : %f ms",avg_jitter[3]);
    syslog(LOG_INFO,"\n\rJitter for Save JPG: %f ms",total_jitter[3]);

    pthread_exit(NULL);
}




/*
delta_t function referred from Dr. Sam Siewert. 
Calculates the execution time.
*/
void delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

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
  return;
}


/*
Function to check the number of frames is equal to 2000 
*/
void frame_check(void)
{
    static int captured_frames = 0;
    if(++captured_frames == 2000)
    {
        capture_flag = FALSE; 
        captured_frames = 0;
    }
}



/*
Sequencer : Referred from seqgen.c by Dr. Sam Siewert
Function is to schedule other services. 
*/
void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,33333333}; // delay for 33.33 msec, 30 Hz
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate

          //modified to schedule at 1Hz
        // Servcie_1 = RT_MAX-1	@ 1 Hz
        if((seqCnt % 30) == 0){
            frame_check();
            sem_post(&semS1);
            
        }    

        // Service_2 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 30) == 0) sem_post(&semS2);

        // Service_3 = RT_MAX-3	@ 1 Hz
        if((seqCnt % 30) == 0) sem_post(&semS3);

        // Service_4 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 30) == 0) sem_post(&semS4);

    } while(capture_flag);

    sem_post(&semS1); sem_post(&semS2);sem_post(&semS3);sem_post(&semS4);
    abortS1=TRUE; abortS2=TRUE; abortS3 = TRUE; abortS4 = TRUE;

    pthread_exit((void *)0);
}





/*
main function: provides entry control into the application. Initialize the frames,
semaphores and other threads. 
*/
int main(void)
{
    openlog("Timelapse",LOG_PERROR,LOG_USER);
    
    clock_gettime(CLOCK_REALTIME,&code_start);
    struct timeval current_time_val;
    int i, rc, scope;
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;

    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    

    cap.set(CV_CAP_PROP_FRAME_WIDTH,HRES);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,VRES);
    cap.set(CV_CAP_PROP_FPS,2000.0);

    cap.open(0);
    ppm_window.create(VRES,HRES,CV_8UC3); //8-bit unsigned integer matrix image with 3 channels RGB
    jpg_window.create(VRES,HRES,CV_8UC3); //8-bit unsigned integer matrix image with 3 channels RGB



   

    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
    //capture, conv_ppm, ppm_complete,time_print,stamp1, conv_jpg
    
    if (sem_init (&conv_ppm,0,1)) { printf ("Failed to initialize ppm\n");exit (-1);}
    if (sem_init (&ppm_complete,0,0)) { printf ("Failed to initialize ppm finish\n");exit (-1);}
    if (sem_init (&time_print,0,1)) { printf ("Failed to initialize time print\n");exit (-1);}
    if (sem_init (&stamp1,0,0)) { printf ("Failed to initialize stamp1\n");exit (-1);}
   
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);
    

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }




    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      capture_frame,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
    {
	    printf("pthread_create successful for service 1\n");
    }

    
    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], save_ppm, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
    {
            printf("pthread_create successful for service 2\n");
    }

    
    rt_param[3].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], save_jpg, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
    {
            printf("pthread_create successful for service 3\n");
    }

    


    
    // Wait for service threads to initialize and await release by sequencer.
    //
    // Note that the sleep is not necessary of RT service threads are created wtih 
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    //
    // usleep(1000000);
 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=900;

    // Sequencer = RT_MAX	@ 30 Hz
    //
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequencer service 0\n");


   
   for(i=0;i<NUM_THREADS;i++)
       pthread_join(threads[i], NULL);

    cap.release();
    clock_gettime(CLOCK_REALTIME,&code_end);

     execution_time = ((code_end.tv_sec - code_start.tv_sec)  + ((code_end.tv_nsec - code_start.tv_nsec)/NSEC_PER_SEC));

    printf("\n\r Timelapse Execution Done ");
}
