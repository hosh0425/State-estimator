#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <state_estimator/Ardrone3PilotingStateSpeedChanged.h>
#include <math.h>

#include <QFile>
#include <QTextStream>



#define delta_t 0.2
#define linear_velocity_noise 5
#define angular_velocity_noise 2

#define alpha_1     0.1
#define alpha_2     0.05
#define alpha_3     0.05
#define alpha_4     0.05

#define range_variance      0.30
#define beam_variance       0.1

class EKF{
public:

    ros::NodeHandle nh;
    ros::Subscriber observation_sub;
    ros::Subscriber motion_command_sub;
    ros::Publisher position;

    struct blief{
        float mean[3];
        float covariance[3][3];
    };

    struct motion_cmd{
        //value of this two variable will change in motion_command call back
        float translational_velocity , rotational_velocity;
    };


    struct observation{
        //value of this three variable will change in observation call back
        float observation_range_x,observation_range_y , observation_beam;
        int marker_id;
    };

    blief prev_position , predicted_position;
    motion_cmd recieved_motion_cmd;
    observation recieved_observation[10];
    int number_of_observation;

    ros::Time begin,end;


    QFile *my_file;
    QString file_name="/home/hossein/Desktop/sec_ekf_log1.txt";
    QTextStream *out;
    QString first_str,second_str,third_str;

    float rand_v_noise,rand_omega_noise;


    //changed : 80
    float marker_pose[85][2]={{8.79,5.81},{8.13,4.25},{8.20,1.68},{8.16,.62},{8.34,-1.03},{8.79,-3.64},{8.78,-4.74},{8.15,-5.73},{8.34,-2.16},{7.49,-5.99},{6.50,-5.99},{5.49,-5.99},//12
                           {4.50,-5.99},{3.64,-5.99},{3.02,-5.28},{1.54,-5.99},{.47,-5.99},{-.57,-5.99},{-1.48,-5.99},{-2.39,-5.99},{-3.08,-5.28},{-4.48,-5.99},{-5.42,-5.99},//23
                           {-6.48,-5.99},{-7.49,-5.99},{-7.99,-5.38},{-7.99,-4.52},{-7.99,-3.46},{-7.99,-2.48},{-7.99,-1.5},{-7.99,-.5},{-7.99,.46},{-7.99,1.54},//33
                           {-7.99,2.5},{-7.99,3.45},{-7.99,4.53},{-7.99,5.58},{-7.44,6.49},{-6.48,6.49},{-5.56,6.49},{-4.2,6.49},{-2.98,5.85},{-1.94,6.49},{-1.04,6.49},{-0.1,6.49},//45
                           {0.9,6.49},{1.9,6.49},{2.94,5.86},{4.09,6.49},{5,6.49},{6,6.49},{7,6.49},{7.98,6.49},{-3.84,1.4},{-3.6,1.14},{-3.54,1.6},{-3.16,.89},{-3.2,1.93}//58
                           ,{-2.91,.63},{-2.85,2.22},{-2.54,.89},{-2.59,1.99},{-2.21,1.18},{-2.18,1.71},{-1.95,1.49},{2.34,1.31},{2.54,.96},{2.63,1.61},{2.85,.61}//69
                           ,{2.94,1.96},{3.06,.34},{3.23,2.23},{3.52,.58},{3.81,.97},{4.10,1.35},{3.86,1.64},{3.47,1.99},{1.755,-0.977},{0.546,-1.01},{-1.43,-2.65},{2.33,4.23},{0.515,4.3},{-0.895,4.31}//83
                           ,{5.90,-2},{-5,-2}
                             };





    EKF();

    void observation_callback(const geometry_msgs::TwistConstPtr &observation_msg);

    void motion_callback(state_estimator::Ardrone3PilotingStateSpeedChanged motion_command_msg);

    void  state_estimator(const ros::TimerEvent& timer);

    float** matrix_transpose_3(float  matrix[][3]);
    float** matrix_transpose_2(float  matrix[][2]);
    float** matrix_transpose_1(float  matrix[][3]);

    float** mult_line_7_step_one(float matrix_1[][3],float** matrix_2);
    float** mult_line_7_step_two(float matrix_1[][2],float matrix_2[][2],float **matrix_3);
    float** mult_line_14(float matrix_1[][3],float** matrix_2);
    float** mat_inverse(float matrix [][2]);
    float** mult_line_15(float** matrix_1,float** matrix_2);
    float** mult_line_16(float** matrix_1,float matrix_2[2]);
    float** mult_line_17(float** matrix_1,float matrix_2[][3]);
    float** mult_line_17_b(float** matrix_1);


    float range;

    float truth=-1.3;
    int repeat=0;
    ros::Timer ros_timer;





};
