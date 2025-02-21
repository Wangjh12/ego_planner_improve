#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

struct Obstacle {
    double x, y;
    double radius, height;
};


struct RingObstacle
{
    double x;
    double y;
    double radius;
    double height;
    double thickness;
};


pcl::KdTreeFLANN<pcl::PointXYZ>
    kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

random_device rd;
// default_random_engine eng(4);
default_random_engine eng(rd()); 

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h;
double _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;

void GenerateObstaclesInA() {
         
    vector<Obstacle> obstacles;

    uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h);
    uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h);
     
    for (int i = 0; i < _obs_num; i++) {
        Obstacle obs;
        double x, y;
        x = rand_x(eng);
        y = rand_y(eng);
        
        /*检查障碍物之间的最小距离，如果过小则重新生成*/
        bool flag_continue = false;
        for ( auto p : obstacles )
            if (Eigen::Vector2d(x-p.x,y-p.y).norm() < 0.5 /*metres*/ )
            {   
                i--;
                flag_continue = true;
                break;
            }
        if ( flag_continue ) continue;

        obs.x = x;
        obs.y = y;
        obs.radius = double(0.25);
        obs.height = double(2.0);
        
        obstacles.push_back(obs);
    }

    pcl::PointXYZ pt;
    for ( int i = 0; i < _obs_num && ros::ok(); i++ ) {

        double x = obstacles[i].x;
        double y = obstacles[i].y;
        double radius = obstacles[i].radius;
        double height = obstacles[i].height;

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        
        int radiusNum = ceil(radius / _resolution);
        int heiNum = ceil(height / _resolution);

        for (int r = -radiusNum / 2.0; r < radiusNum / 2.0; r++) {
            for (int s = -radiusNum / 2.0; s < radiusNum / 2.0; s++) {
                for (int t = 0; t < heiNum; t++) {
                    pt.x = x + (r + 0.5) * _resolution + 1e-2;
                    pt.y = y + (s + 0.5) * _resolution + 1e-2;
                    pt.z = (t + 0.5) * _resolution + 1e-2;
                    
                    if (Eigen::Vector2d(pt.x-x,pt.y-y).norm() <= radius){
                        cloudMap.points.push_back(pt);
                    }           
                }
            }
        }
    }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;



    ROS_WARN("Finished generating specified map");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void GenerateObstaclesInB() {
    pcl::PointXYZ pt;
        double offset = 1.0;
        double length = 6.0;
        double width  = 2.5;
        double height = 2.5;

        //矩形中心点
        double x = length/2 + offset;
        double y = width/2 + offset;
        double radius = double(0.2) ;//层厚
        double z = height/2;

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        
        int lengthNum = ceil(length / _resolution);
        int widthNum  = ceil(width  / _resolution);
        int hightNum  = ceil(height / _resolution);

        for (int l = -lengthNum / 2.0; l < lengthNum / 2.0; l++) {
            for (int w = -widthNum / 2.0; w < widthNum / 2.0; w++) {
                for (int h = -hightNum / 2.0; h < hightNum / 2.0; h++) {
                    pt.x = x + (l + 0.5) * _resolution + 1e-2;
                    pt.y = y + (w + 0.5) * _resolution + 1e-2;
                    pt.z = z + (h + 0.5) * _resolution + 1e-2;
                    if (((pt.x <= (x - length/2 + radius) || pt.x >= (x + length/2 - radius)) || 
                        (pt.y <= (y -  width/2 + radius) || pt.y >= (y +  width/2 - radius)) || 
                        (pt.z <= (z - height/2 + radius) || pt.z >= (z + height/2 - radius))) &&    //集装箱坐标
                        ((pt.y <= (y + 0.2) || pt.y >= (y + 1.0)) || (pt.z >= 2) || (pt.z <= 0.2))) //门坐标
                      {
                        cloudMap.points.push_back(pt);
                      } 
                }
            }
        }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generating specified map");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void GenerateObstaclesInC() {
    pcl::PointXYZ pt;
        double offset = 10.0;
        double length = 5.0;
        double width  = 5.0;
        double height = 3.0;

        //矩形中心点
        double x = width/2 + 1;
        double y = length/2 - offset;
        double radius = double(0.2) ;//层厚
        double z = height/2;

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
       
        int lengthNum = ceil(length / _resolution);
        int widthNum  = ceil(width / _resolution);
        int hightNum  = ceil(height / _resolution);

        for (int l = -lengthNum / 2.0; l < lengthNum / 2.0; l++) {
            for (int w = -widthNum / 2.0; w < widthNum / 2.0; w++) {
                for (int h = -hightNum / 2.0; h < hightNum / 2.0; h++) {
                    pt.x = x + (l + 0.5) * _resolution + 1e-2;
                    pt.y = y + (w + 0.5) * _resolution + 1e-2;
                    pt.z = z + (h + 0.5) * _resolution + 1e-2;

                    bool isBoundary = (pt.x <= (x - length/2 + radius) || pt.x >= (x + length/2 - radius)) || 
                                      (pt.y <= (y - width/2 + radius) || pt.y >= (y + width/2 - radius));
                                    

                    bool isNotGateAorC = (pt.x <= (x + 0.2) || pt.x >= (x + 1.0)) || 
                                         (pt.z >= 2.0);


                    bool isNotGateB = (pt.y >= (y - radius/2) && pt.y <= (y + radius/2)) &&
                                      ((pt.x >= (x - 1.0) || pt.x <= (x - 2.0)) || 
                                       (pt.z >= 2.2) || 
                                       (pt.z <= 1.2));

                    if (isBoundary && isNotGateAorC) {
                        cloudMap.points.push_back(pt);
                    }

                    if (isNotGateB) {
                        cloudMap.points.push_back(pt);
                    }

                }
            }
        }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generating specified map");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void GenerateObstaclesInD() {
    pcl::PointXYZ pt;

        double offset = 15.0;
        double height = 2.5;
        double radius = 1.0;

        //圆环中心点
        double x = offset/5;
        double y = - offset;
        double z = height;

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
 
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
          pt.y = y;
          pt.x = x + radius * cos(angle);
          pt.z = z + radius * sin(angle);

          cloudMap.points.push_back(pt);
        }
    
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generating specified map");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}



void testObstacle() {
    vector<Obstacle> obstacles;
    vector<RingObstacle> ringObstacle;

    // 定义固定障碍物的位置和属性
    Obstacle fixedObstacles[] = {
        {-1.0, 2.0, 0.25, 2.0},
        {-1.5, -6.0, 0.25, 3.0},
        {-3.0, 5.0, 0.25, 3.0},
        {-2.0, 7.0, 0.25, 2.0},
        {-4.0, -5.0, 0.25, 1.0},
        {-3.0, -4.0, 0.25, 2.0},
        {-5.0, 4.0, 0.25, 2.0},
        {-4.0, -3.0, 0.25, 3.0},
        {-6.0, -4.0, 0.25, 2.0},
        {-5.0, -2.0, 0.25, 1.0},
        {-7.0, 3.0, 0.25, 2.0},
        {-6.0, 6.0, 0.25, 3.0},
        {-8.0, -6.0, 0.25, 3.0},
        {-7.0, 7.0, 0.25, 2.0},
        {-8.0, -7.0, 0.25, 2.0},
        {-10.0, -4.0, 0.25, 2.0},
        {-9.0, 6.0, 0.25, 3.0},
        {-11.0, 5.0, 0.25, 3.0},
        {-10.0, -3.0, 0.25, 2.0},
        {-12.0, 1.0, 0.25, 2.0},
        {-11.0, 2.0, 0.25, 3.0},
        {-13.0, -4.0, 0.25, 2.0},
        {-12.0, -2.0, 0.25, 2.0},
        {-14.0, 4.0, 0.25, 3.0},
        {-13.0, 3.0, 0.25, 2.0},
        {-15.0, -6.0, 0.25, 2.0},
        {-14.0, -4.0, 0.25, 3.0},
        {-16.0, -7.0, 0.25, 2.0},
        {-15.0, -0.0, 0.25, 2.0},
        {-12.0, -0.0, 0.25, 2.0},
        {-9.0, -0.0, 0.25, 3.0},
        {-6.0, -0.0, 0.25, 2.0},
        {-3.0, -0.0, 0.25, 3.0},
        };


    int numFixedObstacles = sizeof(fixedObstacles) / sizeof(fixedObstacles[0]);
    for (int i = 0; i < numFixedObstacles; i++) {
        obstacles.push_back(fixedObstacles[i]);
    }

    pcl::PointXYZ pt_random;

    for (int i = 0; i < numFixedObstacles; i++) {
        double x, y, w, h;
        x = obstacles[i].x;
        y = obstacles[i].y;
        w = obstacles[i].radius;


        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil(w / _resolution);

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            h = obstacles[i].height;
            int heiNum = ceil(h / _resolution);
            for (int t = -20; t < heiNum; t++) {
            pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
            pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
            pt_random.z = (t + 0.5) * _resolution + 1e-2;
            cloudMap.points.push_back(pt_random);
            }
        }
    }

//生成垂直障碍物
    Obstacle cuboidObstacle;
    cuboidObstacle.x = -18.0;
    cuboidObstacle.y = 0.0;
    cuboidObstacle.radius = 0.5; // 半径设置为长宽的一半
    cuboidObstacle.height = 0.5;

    double x = cuboidObstacle.x;
    double y = cuboidObstacle.y;
    double w = cuboidObstacle.radius * 2; // 长宽
    double h = cuboidObstacle.height;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);
    int heiNum = ceil(h / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            for (int t = 0; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                    pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                    pt_random.z = 1.0 + (t + 0.5) * _resolution + 1e-2; // 设置高度从2.5开始
                    cloudMap.points.push_back(pt_random);
            }
        }
    }

//生成一面墙
    double wallX = 3.0;
    double wallY = 0.0;
    double wallLength = 1.0;
    double wallWidth = 6.0;
    double wallHeight = 3.0;

    int wallLengthNum = ceil(wallLength / _resolution);
    int wallWidthNum = ceil(wallWidth / _resolution);
    int wallHeightNum = ceil(wallHeight / _resolution);

    for (int r = -wallLengthNum / 2.0; r < wallLengthNum / 2.0; r++) {
        for (int s = -wallWidthNum / 2.0; s < wallWidthNum / 2.0; s++) {
            for (int t = 0; t < wallHeightNum; t++) {
                pt_random.x = wallX + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = wallY + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = (t + 0.5) * _resolution + 1e-2; // 从地面开始生成墙
                cloudMap.points.push_back(pt_random);
            }
        }
    }


        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;

        ROS_WARN("Finished generating specified map");

        kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

        _map_ok = true;
}



void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}



void pubSensedPoints() {

  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);

  return;

  /* ---------- only publish points around current position 发布当前位置周围的局部地图 ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_map_sensing");
    ros::NodeHandle nh("~");
    
    nh.param("map/obs_num", _obs_num, 90);
    nh.param("init_state_x", _init_x, -18.0);
    nh.param("init_state_y", _init_y, 0.0);
    nh.param("map/x_size", _x_size, 40.0);
    nh.param("map/y_size", _y_size, 40.0);
    nh.param("map/z_size", _z_size, 3.0);
    nh.param("sensing/range", _sensing_range, 10.0);
    nh.param("sensing/rate", _sense_rate, 10.0);
    nh.param("map/resolution", _resolution, 0.2);
    nh.param("min_distance", _min_dist, 1.0);
    
    _x_h = _x_size / 2.0;
    _x_l = -_x_size / 2.0;

    _y_h = _y_size / 2.0;
    _y_l = -_y_size / 2.0;
    _state = {_init_x, _init_y, 0.0};

    _local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
    _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

    _odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

    click_map_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  

    ros::Duration(0.5).sleep();


    unsigned int seed = rd();
    // unsigned int seed = 2433201515;
    cout << "seed=" << seed << endl;
    eng.seed(seed);



    // // GenerateObstaclesInA();
    // // GenerateObstaclesInB();
    // // GenerateObstaclesInC();
    // // GenerateObstaclesInD();

    testObstacle();

    ros::Rate loop_rate(_sense_rate);

    while (ros::ok()) {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
