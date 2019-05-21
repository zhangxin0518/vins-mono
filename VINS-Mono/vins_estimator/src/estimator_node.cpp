#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;                //imu buff
queue<sensor_msgs::PointCloudConstPtr> feature_buf;     //feature buff
queue<sensor_msgs::PointCloudConstPtr> relo_buf;        //relocalization buff
int sum_of_wait = 0;        //time of wait 

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;      //位置信息
Eigen::Quaterniond tmp_Q;   //航向四元数
Eigen::Vector3d tmp_V;      //速率信息
Eigen::Vector3d tmp_Ba;     //加速度偏置
Eigen::Vector3d tmp_Bg;     //陀螺仪偏置
Eigen::Vector3d acc_0;      //加速度信息
Eigen::Vector3d gyr_0;      //陀螺仪信息
bool   init_feature = 0;
bool   init_imu = 1;
double last_imu_t = 0;


//*****************************
//预测与处理IMU信息
//预测未考虑观测噪声的p、v、q值，同时将发布最新的IMU测量值消息（pvq值），
//这里计算得到的pvq是估计值，注意是没有观测噪声和偏置的结果，
//作用是与下面预积分计算得到的pvq（考虑了观测噪声和偏置）做差得到残差。
//*****************************
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //初始化IMU
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    //计算时间间隔dt
    double dt = t - latest_time;
    latest_time = t;

    //获取加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    //获取角速率
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    //上一帧的加速度值
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    //预测(估计)的角速率信息(K帧gry 和 K+1帧gry 互补融合)
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    //预测的角速率积分，得到预测(估计)的航向(四元数)
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    //当前帧的加速度值
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    //预测的加速度值(K帧acc 和 K+1帧acc 互补融合)
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    //预测(估计)位姿态信息和预测(估计)速度信息
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    //加速度与陀螺仪信息
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

//*****************************
// update message
//*****************************
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());   //predict

}

//*****************************
//得到IMU和特征点的观测数据
//*****************************
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> 
getMeasurements()
{
    //定义观测数据类：一组IMU + 一帧image
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        //没有IMU和特征点的测量信息
        if (imu_buf.empty() || feature_buf.empty())
        {
            return measurements;
        }

        //判断图像特征数据和IMU数据是否对齐。
        //这里使用的是队列数据结构（先进先出front是先进的数据，back是后进的数据），
        //需要满足两个条件就能保证数据对齐，第一是IMU最后一个数据的时间要大于图像特征最开始数据的时间，
        //第二是IMU最开始数据的时间要小于图像特征最开始数据的时间。
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }

        //满足数据对齐就可以数据从队列中按对齐的方式取出来。
        //这里直到把缓存中的图像特征数据或者IMU数据取完，才能够跳出此函数，返回数据。
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
        {
            ROS_WARN("no imu between two image");
        }

        measurements.emplace_back(IMUs, img_msg);
    }

    return measurements;
}

//*****************************
//IMU回调函数
//IMU位姿预测部分都在这里
//*****************************
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //有问题的IMU信息，返回
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);  //将IMU数据保存到imu_buf中
    m_buf.unlock();
    con.notify_one();       //唤醒作用于process线程中的获取观测值数据的函数

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);   //IMU位姿预测
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            //发布预测的里程计消息
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header); 
        }
    }
}

//*****************************
//特征点回调函数
//*****************************
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    //初始化
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);  //将特征点数据保存到feature_buf中
    m_buf.unlock();
    con.notify_one();               //唤醒作用于process线程中的获取观测值数据的函数
}

//*****************************
//重启追踪估计回调函数
//*****************************
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        //清空feature_buf缓存
        while(!feature_buf.empty())
        {
            feature_buf.pop();
        }
        //清空imu_buf缓存
        while(!imu_buf.empty())
        {
            imu_buf.pop();
        }
        m_buf.unlock();

        m_estimator.lock();
        estimator.clearState();     //清空状态
        estimator.setParameter();   //重新设置参数
        m_estimator.unlock();

        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

//*****************************
//重定位回调函数
//*****************************
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);  //将特征点数据保存到relo_buf中
    m_buf.unlock();
}

//**********************************
//视觉惯性里程计的实现
//**********************************
void process()
{
    //主循环
    while (true)
    {
        //定义观测数据类：一组IMU + 一帧image
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

        //===================================
        //使用了互斥锁和条件等待的功能，互斥锁用来锁住当前代码段，
        //条件等待是等待上面两个接收数据完成就会被唤醒，
        //然后从imu_buf和feature_buf中提取观测数据measurements = getMeasurements()，
        //需要注意的是在提取观测值数据的时候用到的互斥锁会锁住imu_buf和feature_buf等到提取完成才释放
        //整个数据获取的过程是：回调函数接收数据，接收完一组数据唤醒提取数据的线程，
        //提取数据的线程提取完数据后，回调函数就可以继续接收数据，依次往复。这就是线程间通信的曼妙啊！
        //===================================
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0; //得到IMU和特征点的观测数据
                 });
        lk.unlock();

        //===================================
        //处理IMU数据和图像特征数据
        //===================================
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            //图像特征数据
            auto img_msg = measurement.second;

            //--------------------------
            //处理IMU数据
            //--------------------------
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    //优化处理IMU数据
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    //优化处理IMU数据
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            //--------------------------
            //处理重定位数据
            //--------------------------
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            //--------------------------
            //处理图像特征点数据
            //--------------------------
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());
            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            //优化处理图像数据
            estimator.processImage(image, img_msg->header);
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);

            //--------------------------
            //发布各种消息
            //--------------------------
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
            {
                pubRelocalization(estimator);
            }
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();

        //===================================
        //
        //===================================
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            //更新操作
            update();
        }
        m_state.unlock();
        m_buf.unlock();
    }
}


//*******************************************************
//主函数，实现 feature_tracker 和 vins_estimator 的无缝衔接.
//*******************************************************
int main(int argc, char **argv)
{
    //ros主要节点初始化
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //读取配置参数文件
    readParameters(n);

    //设置估计参数
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");
    registerPub(n);

    //订阅相关传感器消息信息
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    //主要的处理线程(主要实现功能)
    std::thread measurement_process{process};

    ros::spin();
    return 0;
}
