#include <state_estimator.h>
#include <state_estimator/Ardrone3PilotingStateSpeedChanged.h>



EKF::EKF(){

    observation_sub=nh.subscribe("/observation",1,&EKF::observation_callback,this);
    motion_command_sub=nh.subscribe("/bebop/states/ardrone3/PilotingState/SpeedChanged",1,&EKF::motion_callback,this);
    position=nh.advertise<geometry_msgs::Twist>("position",100);

    number_of_observation=0;
    prev_position.mean[0]=-1;
    prev_position.mean[1]=1;prev_position.mean[2]=0;
    prev_position.covariance[0][0]=0.1;
    prev_position.covariance[1][1]=0.1;
    prev_position.covariance[2][2]=0.05;

    my_file=new QFile(file_name);
    my_file->open(QFile::WriteOnly |QFile::Text);

    out=new QTextStream(my_file);
    repeat=0;





    prev_position.covariance[0][1]=prev_position.covariance[0][2]=prev_position.covariance[1][0]=prev_position.covariance[1][2]=prev_position.covariance[2][0]=prev_position.covariance[2][1]=0;
    ros_timer = nh.createTimer(ros::Duration(0.2),&EKF::state_estimator,this,true);
    ros_timer.start();

}


void EKF::observation_callback(const geometry_msgs::TwistConstPtr &observation_msg){




        recieved_observation[number_of_observation].observation_range_x=observation_msg->linear.x;
        recieved_observation[number_of_observation].observation_range_y=observation_msg->linear.y;
        recieved_observation[number_of_observation].marker_id=observation_msg->linear.z;
        number_of_observation++;

 ROS_INFO("observation recievecd %d num_observation",number_of_observation);


 if(number_of_observation==9)
     number_of_observation=0;
}


void EKF::motion_callback( state_estimator::Ardrone3PilotingStateSpeedChanged motion_command_msg){

    ROS_INFO("motion_command recieved");
    recieved_motion_cmd.translational_velocity=motion_command_msg.speedX;
    recieved_motion_cmd.rotational_velocity=motion_command_msg.speedX; //**************IN BAYAD AVAZ SHE
    number_of_observation=0;

    ros_timer.start();


}



void EKF::state_estimator(const ros::TimerEvent& timer){
    geometry_msgs::Twist robot_pose_EKF;

    ros_timer.stop();

    ROS_INFO("**********State estimator******************");

    /*
     * line 2
     */

    if(prev_position.mean[2]>3.1415)
        prev_position.mean[2]-=3.1415*2;
    if(prev_position.mean[2]<-3.1415)
        prev_position.mean[2]+=3.1415*2;

    float theta=prev_position.mean[2];






    /*
     * line 3
     */


    float v=recieved_motion_cmd.translational_velocity;
    if(v>0.5 || v<-0.5)
        v=0;
    float omega=recieved_motion_cmd.rotational_velocity;

    repeat++;
    if(repeat==550){
        my_file->close();
        ROS_INFO("log closed");
        exit(0);
    }

    rand_v_noise=rand()%10+1-5;
    rand_v_noise=rand_v_noise/100;

    rand_omega_noise=rand()%4+1-2;
    rand_omega_noise=rand_omega_noise/100;

//    v+=rand_v_noise;
//    omega+=rand_omega_noise;

    ROS_INFO("v: %f    omega:%f",v,omega);

    float motion_jacobian[3][3]={
                                {1   , 0    , (-1*v*delta_t*sin(theta))},
                                {0   , 1    , v*delta_t*cos(theta)},
                                {0   , 0    ,  1}
                                };






    /*
     * line 4
     */
    float V_jacobian[3][2]={
                            {  cos(theta)*delta_t    , 0   },
                            {   sin(theta)*delta_t   ,   0 },
                            {      0 ,  delta_t   }
                            };



    /*
     * line 5
     */
    float M[2][2]={
                    { pow(alpha_1*(float)fabs(v)+alpha_2*(float)fabs(omega),2.0)   ,                       0                     },
                    { 0                                            , pow(alpha_3*(float)fabs(v)+alpha_4*(float)fabs(omega),2.0)  }
                  };




    /*
     * line 6
     */
    predicted_position.mean[0]=prev_position.mean[0] + (v*delta_t*cos(theta));
    predicted_position.mean[1]=prev_position.mean[1] +  (v*delta_t*sin(theta)) ;
    predicted_position.mean[2]=prev_position.mean[2] + omega*delta_t;




    /*
     *line 7
     */
    float ** motion_jacobian_transpose=new float *[3];
    for(int i=0;i<3;i++)
        motion_jacobian_transpose[i]=new float[3];

    float **V_jacobian_transpose=new float *[2];
    for(int i=0;i<2;i++)
        V_jacobian_transpose[i]=new float[3];


    motion_jacobian_transpose=matrix_transpose_3(motion_jacobian);
    V_jacobian_transpose=matrix_transpose_2(V_jacobian);

    //we'll store result of motion_jacobian*prev_covariance*motion_jacobian_transpose in this matrix
    float ** step_one=new float *[3];
    for(int i=0;i<3;i++)
        step_one[i]=new float[3];

    float ** step_two=new float *[3];
    for(int i=0;i<3;i++)
        step_two[i]=new float[3];

    step_one=mult_line_7_step_one(motion_jacobian,motion_jacobian_transpose);
    step_two=mult_line_7_step_two(V_jacobian,M,V_jacobian_transpose);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            predicted_position.covariance[i][j]=step_one[i][j]+step_two[i][j];


    /*
     * line 8
     */

    float Q[2][2]={
                    {range_variance , 0             },
                    {0              , beam_variance }
    };


    /*
     * line 9
     */

    for(int i=0;i<number_of_observation;i++){

        /*
         * line 10
         */
        int marker_id=recieved_observation[i].marker_id;

        /*
         * line 11
         */
        float q=pow(marker_pose[marker_id-1][0]-predicted_position.mean[0],2)+pow(marker_pose[marker_id-1][1]-predicted_position.mean[1],2);


        /*
         * line 12
         */
        float z_hat[2]={sqrt(q),atan2((marker_pose[marker_id-1][1]-predicted_position.mean[1]),(marker_pose[marker_id-1][0]-predicted_position.mean[0]))-predicted_position.mean[2]};


//        first_str.setNum(predicted_position.mean[0]);
//        second_str.setNum(predicted_position.mean[1]);
//        third_str.setNum(predicted_position.mean[2]);
//        *out<<first_str<<"   "<<second_str<<"    "<<third_str<<"\n";
//        my_file->flush();

        if(z_hat[1]>3.1415)
            z_hat[1]-=3.1415*2;
        if(z_hat[1]<-3.1415)
            z_hat[1]+=3.1415*2;

//        ROS_INFO("Z_hat range:  %f     beam:  %f       id:%d ",z_hat[0],z_hat[1],marker_id);
//        ROS_INFO("Z_hat atan2  y:  %f    x:  %f",marker_pose[marker_id-1][1]-predicted_position.mean[1],marker_pose[marker_id-1][0]-predicted_position.mean[0]);






        /*
         * line 13
         */
        float observation_jacobian[2][3]={
                                            {-1*(marker_pose[marker_id-1][0]-predicted_position.mean[0])/sqrt(q)    ,-1*(marker_pose[marker_id-1][1]-predicted_position.mean[1])/sqrt(q) ,0  },
                                            { (marker_pose[marker_id-1][1]-predicted_position.mean[1])/q            ,-1*(marker_pose[marker_id-1][0]-predicted_position.mean[0])/q          , -1 }
        };




        /*
         * line 14
         */
        float **observation_jacobian_transpose=new float*[3];
        for(int i=0;i<3;i++)
            observation_jacobian_transpose[i]=new float[2];

        observation_jacobian_transpose=matrix_transpose_1(observation_jacobian);

        float ** temp=new float *[2];
        for(int i=0;i<2;i++)
            temp[i]=new float[2];

        temp=mult_line_14(observation_jacobian,observation_jacobian_transpose);

        float S[2][2];
        for(int i=0;i<2;i++)
            for(int j=0;j<2;j++)
                S[i][j]=temp[i][j]+Q[i][j];


        /*
         * line 15
         */
        float** s_inverse=new float *[2];
        for(int i=0;i<2;i++)
            s_inverse[i]=new float [2];

        s_inverse=mat_inverse(S);



        float ** K=new float *[3];
        for(int i=0;i<3;i++)
            K[i]=new float[2];
        K=mult_line_15(observation_jacobian_transpose,s_inverse);


         /*
         * line 16
         */

        range=pow(recieved_observation[i].observation_range_x+0.08,2)+pow(recieved_observation[i].observation_range_y,2);
        range=sqrt(range);


        float beam_temp=-1*atan2((recieved_observation[i].observation_range_y),(recieved_observation[i].observation_range_x+0.08));
        if(beam_temp>3.1415)
            beam_temp-=3.1415*2;
        if(beam_temp<-3.1415)
            beam_temp+=3.1415*2;

        float z_mines_z_hat[2]={range-z_hat[0],beam_temp-z_hat[1]};
        ROS_INFO("Z  range:  %f     beam:  %f",range,beam_temp);
        ROS_INFO("Z atan2  y:  %f    x:  %f",recieved_observation[i].observation_range_y,recieved_observation[i].observation_range_x+0.08);



        float ** temp2=new float *[3];
        for(int i=0;i<3;i++)
            temp2[i]=new float[1];

        temp2=mult_line_16(K,z_mines_z_hat);

        for (int i = 0; i < 3; ++i)
            predicted_position.mean[i]+=temp2[i][0];

        /*
         * line 17
         */
        float ** temp3=new float *[3];
        for(int i=0;i<3;i++)
            temp3[i]=new float[3];

        temp3=mult_line_17(K,observation_jacobian);
        temp3[0][0]=1-temp3[0][0];
        temp3[1][1]=1-temp3[1][1];
        temp3[2][2]=1-temp3[2][2];

        temp3[0][1]=-1*temp3[0][1];     temp3[0][2]=-1*temp3[0][2];     temp3[1][0]=-1*temp3[1][0];     temp3[1][2]=-1*temp3[1][2];     temp3[2][0]=-1*temp3[2][0];     temp3[2][1]=-1*temp3[2][1];
        float ** temp4=new float *[3];
        for(int i=0;i<3;i++)
            temp4[i]=new float[3];

        temp4=mult_line_17_b(temp3);

        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                predicted_position.covariance[i][j]=temp4[i][j];


    }




    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            prev_position.covariance[i][j]=predicted_position.covariance[i][j];


    for(int i=0;i<3;i++){
        prev_position.mean[i]=predicted_position.mean[i];
        ROS_INFO(" POSITION%d  %f\n",i,prev_position.mean[i]);
    }

    number_of_observation=0;

    robot_pose_EKF.linear.x=predicted_position.mean[0];
    robot_pose_EKF.linear.y=predicted_position.mean[1];
    robot_pose_EKF.linear.z=1.5;
    robot_pose_EKF.angular.z=predicted_position.mean[2];
    position.publish(robot_pose_EKF);

    first_str.setNum(predicted_position.mean[0]);
    second_str.setNum(predicted_position.mean[1]);
    third_str.setNum(predicted_position.mean[2]);
    *out<<first_str<<"   "<<second_str<<"    "<<third_str<<"\n";
    my_file->flush();


    ROS_INFO("**********THE END******************");





}



float** EKF::matrix_transpose_3(float  matrix[][3]){

    float ** mat_t=new float *[3];
    for(int i=0 ; i<3;i++ )
        mat_t[i]= new float[3];

    for (int i = 0; i < 3; ++i)
        for(int j=0 ; j<3 ; j++)
            mat_t[i][j]=matrix[j][i];

    return mat_t;

}

float** EKF::matrix_transpose_2(float  matrix[][2]){

    float ** mat_t=new float *[2];
    for(int i=0 ; i<3;i++ )
        mat_t[i]= new float[3];

    for (int i = 0; i < 2; ++i)
        for(int j=0 ; j<3 ; j++)
            mat_t[i][j]=matrix[j][i];

    return mat_t;

}


float** EKF::mult_line_7_step_one(float matrix_1[][3],float** matrix_2){

    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[3];

    float ** mult2=new float *[3];
    for(int i=0;i<3;i++)
        mult2[i]=new float[3];

    float sum=0;


    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=matrix_1[i][k]*matrix_2[k][j];

            mult1[i][j]=sum;
        }

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=mult1[i][k]*prev_position.covariance[k][j];

            mult2[i][j]=sum;
        }

    return mult2;


}




float** EKF::mult_line_7_step_two(float matrix_1[][2],float matrix_2[][2],float **matrix_3){

    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[2];

    float ** mult2=new float *[3];
    for(int i=0;i<3;i++)
        mult2[i]=new float[3];

    float sum=0;

    for(int i=0;i<3;i++)
        for(int j=0;j<2;j++){
            sum=0;
            for(int k=0;k<2;k++)
                sum+=matrix_1[i][k]*matrix_2[k][j];

            mult1[i][j]=sum;
        }

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<2;k++)
                sum+=mult1[i][k]*matrix_3[k][j];

            mult2[i][j]=sum;
        }

    return mult2;

}

float** EKF::matrix_transpose_1(float  matrix[][3]){

    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[2];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            mult1[i][j]=matrix[j][i];

    return mult1;
}



float** EKF::mult_line_14(float matrix_1[][3],float** matrix_2){

    float ** mult1=new float *[2];
    for(int i=0;i<2;i++)
        mult1[i]=new float[3];

    float ** mult2=new float *[2];
    for(int i=0;i<2;i++)
        mult2[i]=new float[2];

    float sum=0;


    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=matrix_1[i][k]*predicted_position.covariance[k][j];

            mult1[i][j]=sum;
        }

    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=mult1[i][k]*matrix_2[k][j];

            mult2[i][j]=sum;
        }

    return mult2;


}


float** EKF::mat_inverse(float matrix [][2]){

    float ** mat_inv=new float *[2];
    for(int i=0;i<2;i++)
        mat_inv[i]=new float[2];

    float determinant= (matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1] );





            mat_inv[0][0]=matrix[1][1]/determinant;
            mat_inv[0][1]=-1*matrix[0][1]/determinant;
            mat_inv[1][0]=-1*matrix[1][0]/determinant;
            mat_inv[1][1]=matrix[0][0]/determinant;

    return mat_inv;
}




float** EKF::mult_line_15(float** matrix_1,float** matrix_2){
    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[2];

    float ** mult2=new float *[3];
    for(int i=0;i<3;i++)
        mult2[i]=new float[2];

    float sum=0;


    for(int i=0;i<3;i++)
        for(int j=0;j<2;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=predicted_position.covariance[i][k]*matrix_1[k][j];

            mult1[i][j]=sum;
        }

    for(int i=0;i<3;i++)
        for(int j=0;j<2;j++){
            sum=0;
            for(int k=0;k<2;k++)
                sum+=mult1[i][k]*matrix_2[k][j];

            mult2[i][j]=sum;
        }
//    ROS_INFO("MULT1");
//    for(int i=0;i<3;i++)
//        for(int j=0;j<2;j++)
//            ROS_INFO("%f",mult1[i][j]);

//    ROS_INFO("MULT 2");
//    for(int i=0;i<3;i++)
//        for(int j=0;j<2;j++)
//            ROS_INFO("%f",mult2[i][j]);

    return mult2;

}


float** EKF::mult_line_16(float** matrix_1,float matrix_2[2]){
    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[1];

    mult1[0][0]=matrix_1[0][0]*matrix_2[0]+matrix_1[0][1]*matrix_2[1];
    mult1[1][0]=matrix_1[1][0]*matrix_2[0]+matrix_1[1][1]*matrix_2[1];
    mult1[2][0]=matrix_1[2][0]*matrix_2[0]+matrix_1[2][1]*matrix_2[1];

    return mult1;
}


float** EKF::mult_line_17(float** matrix_1,float matrix_2[][3]){
    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[3];

    float sum=0;

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<2;k++)
                sum+=matrix_1[i][k]*matrix_2[k][j];

            mult1[i][j]=sum;
        }

    return mult1;
}


float** EKF::mult_line_17_b(float** matrix_1){
    float ** mult1=new float *[3];
    for(int i=0;i<3;i++)
        mult1[i]=new float[3];



    float sum=0;


    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            sum=0;
            for(int k=0;k<3;k++)
                sum+=matrix_1[i][k]*predicted_position.covariance[k][j];

            mult1[i][j]=sum;
        }


    return mult1;

}


