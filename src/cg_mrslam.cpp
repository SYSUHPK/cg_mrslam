// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/stuff/command_args.h"

#include <string>
#include <sstream> 

#include "mrslam/mr_graph_slam.h"
#include "mrslam/graph_comm.h"
#include "ros_utils/ros_handler.h"
#include "ros_utils/graph_ros_publisher.h"

#include "ros_map_publisher/graph2occupancy.h"
#include "ros_map_publisher/occupancy_map_server.h"

using namespace g2o;

int main(int argc, char **argv)
{
  // g2o中的类，变量设置
  CommandArgs arg;
  double resolution;
  double maxScore, maxScoreMR;
  double kernelRadius;
  int  minInliers, minInliersMR;
  int windowLoopClosure, windowMRLoopClosure;
  double inlierThreshold;
  int idRobot;
  int nRobots;
  std::string base_addr;
  std::string outputFilename;
  std::string odometryTopic, scanTopic, mapTopic, odomFrame, mapFrame, baseFrame;
  std::vector<double> initialPose;
  initialPose.clear();
  bool publishMap, publishGraph;

  float localizationAngularUpdate, localizationLinearUpdate;
  float maxRange, usableRange, infinityFillingRange;

  std::string modality;
  TypeExperiment typeExperiment;
  // key-value default notes
  // 从指令中读取数据设置
  arg.param("resolution",  resolution, 0.025, "resolution of the matching grid");
  arg.param("maxScore",    maxScore, 0.15,     "score of the matcher, the higher the less matches");
  arg.param("kernelRadius", kernelRadius, 0.2,  "radius of the convolution kernel");
  arg.param("minInliers",    minInliers, 7,     "min inliers");
  arg.param("windowLoopClosure",  windowLoopClosure, 10,   "sliding window for loop closures");
  arg.param("inlierThreshold",  inlierThreshold, 2.,   "inlier threshold");
  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots" );
  arg.param("baseAddr", base_addr, "192.168.0.", "base IP address of the MR system" );
  arg.param("angularUpdate", localizationAngularUpdate, M_PI_4, "angular rotation interval for updating the graph, in radians");
  arg.param("linearUpdate", localizationLinearUpdate, 0.25, "linear translation interval for updating the graph, in meters");
  arg.param("maxScoreMR",    maxScoreMR, 0.15,  "score of the intra-robot matcher, the higher the less matches");
  arg.param("minInliersMR",    minInliersMR, 5,     "min inliers for the intra-robot loop closure");
  arg.param("windowMRLoopClosure",  windowMRLoopClosure, 10,   "sliding window for the intra-robot loop closures");
  arg.param("odometryTopic", odometryTopic, "odom", "odometry ROS topic");
  arg.param("scanTopic", scanTopic, "scan", "scan ROS topic");
  arg.param("mapTopic", mapTopic, "map", "map ROS topic");
  arg.param("odomFrame", odomFrame, "odom", "odom frame");
  arg.param("mapFrame", mapFrame, "map", "map frame");
  arg.param("baseFrame", baseFrame, "/base_link", "base robot frame");
  arg.param("initialPose", initialPose, std::vector<double>(), "Pose of the first vertex in the graph. Usage: -initial_pose 0,0,0");
  arg.param("publishMap", publishMap, false, "Publish map");
  arg.param("publishGraph", publishGraph, false, "Publish graph");
  arg.param("modality", modality, "sim", "Choose mrslam modality from [sim, real, bag]. Consult Readme for differences between modalities.");
  arg.param("o", outputFilename, "", "file where to save output");
  // 赋值载入
  arg.parseArgs(argc, argv);
  
  // 模式设置
  if (modality != "sim" && modality != "real" && modality != "bag"){
    std::cerr << "Unknown modality: " << modality << std::endl;
    exit(0);
  } else {
    std::cerr << "Starting cg_mrslam in modality: " << modality << std::endl;
    if (modality == "sim")
      typeExperiment = SIM;
    else if (modality == "real")
      typeExperiment = REAL;
    else
      typeExperiment = BAG;
  }
  
  //map parameters
  // map的参数设置
  float mapResolution = 0.05;
  float occupiedThreshold = 0.65; 
  float rows = 0;
  float cols = 0;
  float gain = 3.0;
  float squareSize = 0;
  float angle = M_PI_2;
  float freeThreshold = 0.196;

  // nodename，可以作为私有命名空间
  ros::init(argc, argv, "cg_mrslam");
  // 初始化了一系列的东西，设置一系列的位姿
  RosHandler rh(idRobot, nRobots, typeExperiment);
  rh.setOdomTopic(odometryTopic);
  rh.setScanTopic(scanTopic);
  rh.setBaseFrame(baseFrame);
  rh.useOdom(true);
  rh.useLaser(true);
  // 初始化odom，scan，base
  rh.init();   //Wait for initial odometry and laserScan
  // 根据三种模式，订阅和发布不同的信息
  rh.run();
  // 获得laser的最大range
  maxRange = rh.getLaserMaxRange();
  usableRange = maxRange;
  infinityFillingRange = 5.0;

  //For estimation
  // 使用g2o进行优化计算
  SE2 currEst;
  // 获取odompose
  SE2 odomPosk_1 = rh.getOdom();
  // 判断初始位姿合理性
  if (initialPose.size()){
    if (initialPose.size()==3){
      currEst = SE2(initialPose[0],initialPose[1],initialPose[2]);
    }else {
      std::cerr << "Error. Provide a valid initial pose (x, y, theta)" << std::endl;
      exit(0);
    }
  }else{
  // 如果是SIM模式，没有初始位姿，需要获取GroundTruth
    if (typeExperiment == SIM)
      currEst = rh.getGroundTruth(idRobot);
    // 初始化pos和odom一样
    else
      currEst = odomPosk_1;
  }
  
  std::cout << "My initial position is: " << currEst.translation().x() << " " << currEst.translation().y() << " " << currEst.rotation().angle() << std::endl;
  std::cout << "My initial odometry is: " << odomPosk_1.translation().x() << " " << odomPosk_1.translation().y() << " " << odomPosk_1.rotation().angle() << std::endl;

  //Graph building
  // 点云和轨迹，用于显示
  // 与InterRobot相关
  MRGraphSLAM gslam;
  // id
  gslam.setIdRobot(idRobot);
  // base id
  int baseId = 10000;
  gslam.setBaseId(baseId);
  // 优化初始化，包括分辨率，内核，闭环检测的滑动窗口过，最大score，内群阈值，最小内群边数
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);
  // 设置closure参数
  gslam.setInterRobotClosureParams(maxScoreMR, minInliersMR, windowMRLoopClosure);
  // 设置laser的一些参数并获取
  RobotLaser* rlaser = rh.getLaser();
  // 设置laser初始化Data
  gslam.setInitialData(currEst, rlaser);
  // 占用地图和地图中心
  cv::Mat occupancyMap;
  Eigen::Vector2f mapCenter;
  
  //Map building
  // 网格占用地图，用于merge
  // 将graph转为occupancy map
  Graph2occupancy mapCreator(gslam.graph(), &occupancyMap, currEst, mapResolution, occupiedThreshold, rows, cols, maxRange, usableRange, infinityFillingRange, gain, squareSize, angle, freeThreshold);
  // 用于保存map
  OccupancyMapServer mapServer(&occupancyMap, idRobot, mapFrame, mapTopic, occupiedThreshold, freeThreshold);
  // 用于用于发布graph
  GraphRosPublisher graphPublisher(gslam.graph(), mapFrame, odomFrame);
  // 发布map和graph
  if (publishMap){
    mapCreator.computeMap();
    
    mapCenter = mapCreator.getMapCenter();
    mapServer.setOffset(mapCenter);
    mapServer.setResolution(mapResolution);
    mapServer.publishMapMetaData();
    mapServer.publishMap();
  }

  if (publishMap || publishGraph){
    graphPublisher.start();
    graphPublisher.setEstimate(gslam.lastVertex()->estimate());
    graphPublisher.setOdom(odomPosk_1);
  }
  
  if (publishGraph)
    graphPublisher.publishGraph();
  
  //Saving g2o file
  // 保存g2o文件 graph
  char buf[100];
  sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
  ofstream ofmap(buf);
  gslam.graph()->saveVertex(ofmap, gslam.lastVertex());

  ////////////////////
  //Setting up network
  // 设置网络
  GraphComm gc(&gslam, idRobot, nRobots, base_addr, typeExperiment);
  gc.init_network(&rh);
  // 重复操作
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    // 获取当前odom
    SE2 odomPosk = rh.getOdom(); //current odometry
    // 上一个odom*现在的 = 真实的
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;
    // 赋值
    odomPosk_1 = odomPosk;
    // 达到更新条件
    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > localizationLinearUpdate) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > localizationAngularUpdate)){
      //Add new data
      // 添加新数据 找约束 优化
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(currEst, laseri);
      gslam.findConstraints();
      gslam.findInterRobotConstraints();

      gslam.optimize(5);
      // 赋值
      currEst = gslam.lastVertex()->estimate();
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
      // 保存graph
      gslam.saveGraph(buf);
      // 设置
      if (publishMap || publishGraph){
	graphPublisher.setEstimate(gslam.lastVertex()->estimate());
	graphPublisher.setOdom(odomPosk_1);
      }

      //Publish graph to visualize it on Rviz
      // 发布
      if (publishGraph)
	graphPublisher.publishGraph();
      // 更新发布
      if (publishMap){
	//Update map
        mapCreator.computeMap();
        mapCenter = mapCreator.getMapCenter();
        mapServer.setOffset(mapCenter);
        mapServer.publishMapMetaData();
	mapServer.publishMap();
      }
      
    }else {
      //Publish map transform with last corrected estimate + odometry drift
      if (publishMap || publishGraph){
	graphPublisher.setEstimate(currEst);
	graphPublisher.setOdom(odomPosk_1);
      }
    }
    
    loop_rate.sleep();
  }
  // 优化保存
  cerr << "Last Optimization...";
  gslam.optimize(5);
  gslam.saveGraph(buf);
  cerr << "Done" << endl;

  if (publishMap || publishGraph)
    graphPublisher.stop();

  return 0;
}
