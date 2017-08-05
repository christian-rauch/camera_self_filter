/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * robotMeshModel.h
 *
 *  Created on: Oct 7, 2010
 *      Author: Christian Bersch
 */

#ifndef ROBOTMESHMODEL_H_
#define ROBOTMESHMODEL_H_


#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

//#include <assimp/assimp.hpp>
#include <assimp/Importer.hpp>
//#include <assimp/aiScene.h>
#include <assimp/scene.h>
//#include <assimp/aiPostProcess.h>
#include <assimp/postprocess.h>
//#include <assimp/IOStream.h>
#include <assimp/IOStream.hpp>
//#include <assimp/IOSystem.h>
#include <assimp/IOSystem.hpp>

#include <resource_retriever/retriever.h>

class RobotMeshModel
{
public:



    tf::TransformListener tf_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

	sensor_msgs::CameraInfoConstPtr cam_info_;
	ros::ServiceServer srver_;
	urdf::Model urdf_;
	std::string description_path;

	std::string modelframe_;
	std::string cameraframe_;
	std::string camera_topic_;
	std::string camera_info_topic_;

	tf::StampedTransform cameraPose;
    bool use_assimp_;

    std::map<std::string, const aiScene *> as_meshes;
	std::map<std::string, tf::Transform > offsets_;
	std::map<std::string, tf::StampedTransform > robotLinks_;
    std::vector<boost::shared_ptr<urdf::Link> > links_with_meshes;

    ros::Time current_time_stamp_;


    RobotMeshModel();
    ~RobotMeshModel();


    void setCamera();
    void setCameraInfo(sensor_msgs::CameraInfoConstPtr cam_info);
    bool setCameraPose();
    void initRobot();
    void updateRobotLinks(const ros::Time time_stamp);
    void paintRobot();


  //from Nico Blodow for now
  struct Vertex
  {
    float x,y,z;
    float nx,ny,nz;
    Vertex ()
    {}
    ~Vertex ()
    {}
    Vertex (float x, float y, float z, float nx, float ny, float nz)
      : x(x), y(y), z(z), nx(nx), ny(ny), nz(nz)
    {}
  };
  
  struct SubMesh
  {
    enum {INVALID_VALUE = 0xFFFFFFFF};
    SubMesh ();
    ~SubMesh ();
    // void init (const std::vector<Vertex>& vertices,
    //            const std::vector<unsigned int>& indices);
    //GLuint vbo, ibo;
    unsigned int num_indices;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
  };

  void fromAssimpScene (const aiScene* scene);
  void initMesh (const aiMesh* mesh);

  std::vector<SubMesh> meshes_vec;

};



#endif /* ROBOTMESHMODEL_H_ */
