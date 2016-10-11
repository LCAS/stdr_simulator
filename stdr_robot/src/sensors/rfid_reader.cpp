/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#include <stdr_robot/sensors/rfid_reader.h>

namespace stdr_robot {
  
  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::SonarSensorMsg&] The sonar description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  RfidReader::RfidReader(const nav_msgs::OccupancyGrid& map,
      const stdr_msgs::RfidSensorMsg& msg, 
      const std::string& name,
      ros::NodeHandle& n)
  : 
    Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency),rng(), var_nor(rng, boost::normal_distribution<>(0.0, 1.0))
  {
    _description = msg;

    _publisher = n.advertise<stdr_msgs::RfidSensorMeasurementMsg>
      ( _namespace + "/" + msg.frame_id, 1 );
      
    rfids_subscriber_ = n.subscribe(
      "stdr_server/rfid_list", 
      1, 
      &RfidReader::receiveRfids,
      this);
      
      
  }
  
  /**
  @brief Default destructor
  @return void
  **/ 
  RfidReader::~RfidReader(void)
  {
    
  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void RfidReader::updateSensorCallback() 
  {
    if (rfid_tags_.rfid_tags.size() == 0) return;    

    stdr_msgs::RfidSensorMeasurementMsg measuredTagsMsg;

    measuredTagsMsg.header.frame_id = _description.frame_id;

    
    float max_range = _description.maxRange;
    float sensor_th = tf::getYaw(_sensorTransform.getRotation());
    float min_angle = sensor_th - _description.angleSpan / 2.0;
    float max_angle = sensor_th + _description.angleSpan / 2.0;
    
    //!< Must implement the functionality
    for(unsigned int i = 0 ; i < rfid_tags_.rfid_tags.size() ; i++)
    {
      //!< Check for max distance
      float sensor_x = _sensorTransform.getOrigin().x();
      float sensor_y = _sensorTransform.getOrigin().y();
      float dist = sqrt(
        pow(sensor_x - rfid_tags_.rfid_tags[i].pose.x, 2) +
        pow(sensor_y - rfid_tags_.rfid_tags[i].pose.y, 2)
      );
      if(dist > max_range)
      {
        continue;
      }
      
      //!< Check for correct angle
      float ang = atan2(rfid_tags_.rfid_tags[i].pose.y - sensor_y,
        rfid_tags_.rfid_tags[i].pose.x - sensor_x);
      
      if(!stdr_robot::angCheck(ang, min_angle, max_angle))
      {
        continue;
      }
      
      float rPow=receivedPower(rfid_tags_.rfid_tags[i]);
            
      measuredTagsMsg.rfid_tags_ids.push_back(rfid_tags_.rfid_tags[i].tag_id);
      measuredTagsMsg.rfid_tags_msgs.push_back(rfid_tags_.rfid_tags[i].message);
      measuredTagsMsg.rfid_tags_dbs.push_back(rPow); 
    }
    
    measuredTagsMsg.header.stamp = ros::Time::now();
    measuredTagsMsg.header.frame_id = _namespace + "_" + _description.frame_id;
    _publisher.publish( measuredTagsMsg );
  }
  
/////////////////////////////////////////////////////////////////
  float RfidReader::receivedPower(const stdr_msgs::RfidTag rfid_tag ){
    float receivedPower=0.0;
    float av_power=0.0;
    float std_power=1.0;
    
    //constants
    float av_max=-25.0;
    float max_loss_dist=40.0;
    float max_loss_ang=10.0;

    
    // reader data
    float max_range = _description.maxRange;
    float sensor_th = tf::getYaw(_sensorTransform.getRotation());
    float min_angle = sensor_th - _description.angleSpan / 2.0;
    float max_angle = sensor_th + _description.angleSpan / 2.0;    
    float sensor_x = _sensorTransform.getOrigin().x();
    float sensor_y = _sensorTransform.getOrigin().y();
    
    // relative positions
    float dist = sqrt(pow(sensor_x - rfid_tag.pose.x, 2) +
                      pow(sensor_y - rfid_tag.pose.y, 2) );
      
    float ang = atan2(rfid_tag.pose.y - sensor_y,
                      rfid_tag.pose.x - sensor_x);
    // We determine two parameters average power and std and use db ~ N(av,std)
    
    // av power is f(distance,angle)     
    av_power = av_max - max_loss_dist*(dist/max_range)-max_loss_ang*(ang/(max_angle-min_angle));
        
    // std is fixed to 1 by now.    
    std_power=1.0;    
         
    receivedPower=av_power+(std_power*var_nor());
    
    
    return receivedPower;
  }
/////////////////////////////////////////////////////////////////
  
  /**
  @brief Receives the existent rfid tags
  **/
  void RfidReader::receiveRfids(const stdr_msgs::RfidTagVector& msg)
  {
    rfid_tags_ = msg;
  }

}  // namespace stdr_robot
