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
 * robotMeshModel.cpp
 *
 *  Created on: Oct 7, 2010
 *      Author: Christian Bersch
 */


#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include "camera_self_filter/robotMeshModel.h"
#include "LinearMath/btTransform.h"

class ResourceIOStream : public Assimp::IOStream
{
public:
  ResourceIOStream(const resource_retriever::MemoryResource& res)
    : res_(res)
    , pos_(res.data.get())
  {}

  ~ResourceIOStream()
  {}

  size_t Read(void* buffer, size_t size, size_t count)
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size)
      {
        to_read = res_.size - (pos_ - res_.data.get());
      }

    memcpy(buffer, pos_, to_read);
    pos_ += to_read;

    return to_read;
  }

  size_t Write( const void* buffer, size_t size, size_t count) {
    ROS_BREAK(); return 0; }

  aiReturn Seek( size_t offset, aiOrigin origin)
  {
    uint8_t* new_pos = 0;
    switch (origin)
      {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset; // TODO is this right?  can offset  really not be negative
    break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset; // TODO is   this right?
    break;
      default:
        ROS_BREAK();
      }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() +
    res_.size)
      {
        return aiReturn_FAILURE;
      }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const
  {
    return res_.size;
  }

  void Flush() {}

private:
  resource_retriever::MemoryResource res_;
  uint8_t* pos_;
};

class ResourceIOSystem : public Assimp::IOSystem
{
public:
  ResourceIOSystem()
  {
  }

  ~ResourceIOSystem()
  {
  }

  // Check whether a specific file exists
  bool Exists(const char* file) const
  {
    // Ugly -- two retrievals where there should be one (Exists + Open)
  // resource_retriever needs a way of checking for existence
  // TODO: cache this
  resource_retriever::MemoryResource res;
      try 
        {   
          res = retriever_.get(file);
        }   
      catch (resource_retriever::Exception& e)
        {   
          return false;
        }   

  return true;
}

// Get the path delimiter character we'd like to see
  char getOsSeparator() const
  {
    return '/';
  }

// ... and finally a method to open a custom stream
Assimp::IOStream* Open(const char* file, const char* mode)
{
  ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

  // Ugly -- two retrievals where there should be one (Exists + Open)
  // resource_retriever needs a way of checking for existence
  resource_retriever::MemoryResource res;
      try 
        {   
          res = retriever_.get(file);
        }   
      catch (resource_retriever::Exception& e)
        {   
          return 0;
        }   

      return new ResourceIOStream(res);
}

void Close(Assimp::IOStream* stream) { delete stream; }

private:
mutable resource_retriever::Retriever retriever_;
};


RobotMeshModel::SubMesh::SubMesh ()
{
  // vbo = INVALID_VALUE;
  // ibo = INVALID_VALUE;
  // num_indices = 0;
}

RobotMeshModel::SubMesh::~SubMesh ()
{
  // if (vbo != INVALID_VALUE)
  //   glDeleteBuffers (1, &vbo);
  // if (ibo != INVALID_VALUE)
  //   glDeleteBuffers (1, &ibo);
}

void RobotMeshModel::fromAssimpScene (const aiScene* scene)
{
  //  meshes_vec.resize (scene->mNumMeshes);
  // std::cerr << "meshes loaded: " << meshes_vec.size () << std::endl;
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
    {
      const aiMesh* mesh = scene->mMeshes[i];
      initMesh (mesh);
    }
}

void RobotMeshModel::initMesh (const aiMesh* mesh)
{
  // TODO: mesh->mMaterialIndex
  // TODO: const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
    {
      const aiVector3D* pos = &(mesh->mVertices[i]);
      const aiVector3D* n = &(mesh->mNormals[i]);
      // TODO: const aiVector3D* pTexCoord = mesh->HasTextureCoords(0) ? &(mesh->mTextureCoords[0][i]) : &Zero3D;

      Vertex v(pos->x, pos->y, pos->z, n->x, n->y, n->z);
      vertices.push_back (v);
    }

  for (unsigned int i = 0 ; i < mesh->mNumFaces ; ++i)
    {
      const aiFace& face = mesh->mFaces[i];
      assert(face.mNumIndices == 3);
      indices.push_back(face.mIndices[0]);
      indices.push_back(face.mIndices[1]);
      indices.push_back(face.mIndices[2]);
    }

  SubMesh submesh;
  submesh.vertices.resize (vertices.size()); 
  submesh.vertices = vertices;
  submesh.indices.resize (indices.size()); 
  submesh.indices = indices;
  meshes_vec.push_back (submesh);
  // meshes[index].init (vertices, indices);
}

RobotMeshModel::RobotMeshModel():nh_priv_("~"), use_assimp_ (true)
{

  //Load robot description from parameter server
  std::string robot_desc_string;
  if(!nh_.getParam("robot_description", robot_desc_string))
    {
      ROS_ERROR("Could not get urdf from param server");
    }
  
  if (!urdf_.initString(robot_desc_string))
    {
      ROS_ERROR("Failed to parse urdf");  
    }
  //modelframe_= "/torso_lift_link";
  modelframe_= "/base";
  
  nh_priv_.param<std::string>("robot_description_package_path", description_path, "..");
  ROS_INFO("package_path %s", description_path.c_str());
  nh_priv_.param<std::string>("camera_topic", camera_topic_, "/wide_stereo/right" );
  nh_priv_.param<std::string>("camera_info_topic", camera_info_topic_, "/wide_stereo/right/camera_info" );
  
  
  //Load robot mesh for each link
  std::vector<boost::shared_ptr<urdf::Link> > links ;
  urdf_.getLinks(links);
  for (int i=0; i< links.size(); i++){
    if (links[i]->visual.get() == NULL) continue;
    if (links[i]->visual->geometry.get() == NULL) continue;
    if (links[i]->visual->geometry->type == urdf::Geometry::MESH)
      {
        //todo: this should really be done by resource retriever
        boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (links[i]->visual->geometry);
        std::string filename (mesh->filename);
        std::string filename_resource = filename;
        
        if (filename.substr(filename.size() - 4 , 4) != ".stl" && filename.substr(filename.size() - 4 , 4) != ".dae") continue;
        
        // TODO load with assimp
        if (filename.substr(filename.size() - 4 , 4) == ".dae")
          filename.replace(filename.size() - 4 , 4, ".stl");
        //ROS_INFO("adding link %d %s",i,links[i]->name.c_str());
        filename.erase(0,25);
        filename = description_path + filename;
        
        Assimp::Importer as_importer;
        as_importer.SetIOHandler(new ResourceIOSystem());
        const aiScene* scene =
          as_importer.ReadFile(filename_resource.c_str(), aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
        
        if (!scene)
          {
            ROS_ERROR("Could not load resource [%s]:  %s", filename_resource.c_str(),
                      as_importer.GetErrorString());
          }
        else
          {
            ROS_INFO ("Mesh [%s] successfully loaded.",
                      filename_resource.c_str());
            if (!scene->HasMeshes())
              { 
                ROS_ERROR("No meshes found in file [%s]", filename_resource.c_str());
              }
            std::cerr << "Constructor mNumMeshes: " << scene->mNumMeshes << std::endl;
          }
        fromAssimpScene (scene);
        as_meshes[links[i]->name] = scene;
      }
    links_with_meshes.push_back(links[i]);
    
    
    tf::Vector3 origin(links[i]->visual->origin.position.x, links[i]->visual->origin.position.y, links[i]->visual->origin.position.z);
    tf::Quaternion rotation(links[i]->visual->origin.rotation.x, links[i]->visual->origin.rotation.y, links[i]->visual->origin.rotation.z, links[i]->visual->origin.rotation.w);
    
    offsets_[links[i]->name] = tf::Transform(rotation,
                                             origin);
  }
  
  initRobot();
  
  //get camera intinsics
  ROS_INFO("waiting for %s", camera_info_topic_.c_str());
  cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
  cameraframe_ = cam_info_->header.frame_id;
  
  ROS_INFO("%s: robot model initialization done!", ros::this_node::getName().c_str());
}

RobotMeshModel::~RobotMeshModel(){

}

//TODO: Remove
void RobotMeshModel::initRobot()
{
};


void RobotMeshModel::updateRobotLinks(const ros::Time time_stamp){

	// get the current configuration of the robot links
	if (current_time_stamp_ != time_stamp){
		current_time_stamp_ = time_stamp;
		tf::StampedTransform tf;
		for (int i=0; i < links_with_meshes.size(); ++i){
//			if (!tf_.waitForTransform("/"+links_with_meshes[i]->name, modelframe_, current_time_stamp_, ros::Duration(0.5))){
//				ROS_ERROR("could not get transform from %s to %s", links_with_meshes[i]->name.c_str(),modelframe_.c_str() );
//				continue;
//			}
//			tf_.lookupTransform( modelframe_, links_with_meshes[i]->name, current_time_stamp_, tf);
            try{
                tf_.lookupTransform( modelframe_, links_with_meshes[i]->name, time_stamp, tf);
            }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              //ros::Duration(1.0).sleep();
            }

			tf  *= offsets_[links_with_meshes[i]->name];
			robotLinks_[links_with_meshes[i]->name] = tf;

		}

	}
}


void RobotMeshModel::paintRobot()
{
  // draw the configuration of the robot links as  specified in robotLinks_
  float d=1.0f; ///mesh.bbox.Diag();
  glScalef(1.0, 1.0, 1.0);
  glDisable(GL_CULL_FACE);
  glMatrixMode(GL_MODELVIEW);
  std::cerr << "meshes: " << meshes_vec.size () << std::endl;
  for (unsigned int n = 0; n < meshes_vec.size (); n++) 
    {
      std::cerr << "Lin name: " << links_with_meshes[n]->name <<
        " Vertices: " << meshes_vec[n].vertices.size() << std::endl;
//      btScalar glTf[16];
      tfScalar *glTf;
      glTf = new tfScalar[16];
      //robotLinks_[links_with_meshes[n]->name].getOpenGLMatrix(glTf);
      robotLinks_[links_with_meshes[n]->name].getOpenGLMatrix(glTf);
      glPushMatrix();
      glMultMatrixd((GLdouble*)glTf);
      
      for (unsigned int t = 0; t < meshes_vec[n].indices.size(); t
             = t + 3)
        {
          GLenum face_mode;
          switch(3) 
            {
            case 1: face_mode =
                GL_POINTS; break;
            case 2: face_mode =
                GL_LINES; break;
            case 3: face_mode =
                GL_TRIANGLES; break;
            default: face_mode
                = GL_POLYGON; break;
            }
          glBegin(face_mode); // Drawing Using Triangles
          glVertex3f(meshes_vec[n].vertices[t].x,
                     meshes_vec[n].vertices[t].y,
                     meshes_vec[n].vertices[t].z); // Top
          glVertex3f(meshes_vec[n].vertices[t+1].x,
                     meshes_vec[n].vertices[t+1].y,
                     meshes_vec[n].vertices[t+1].z); // Bottom Left
          glVertex3f(meshes_vec[n].vertices[t+2].x,
                     meshes_vec[n].vertices[t+2].y,
                     meshes_vec[n].vertices[t+2].z); // Bottom Right
          
          glNormal3f(meshes_vec[n].vertices[t].nx,
                     meshes_vec[n].vertices[t].ny,
                     meshes_vec[n].vertices[t].nz); // Top
          glNormal3f(meshes_vec[n].vertices[t+1].nx,
                     meshes_vec[n].vertices[t+1].ny,
                     meshes_vec[n].vertices[t+1].nz); // Bottom Left
          glNormal3f(meshes_vec[n].vertices[t+2].nx,
                     meshes_vec[n].vertices[t+2].ny,
                     meshes_vec[n].vertices[t+2].nz); // Bottom Right
          glEnd();  
        }
      glPopMatrix();
      
    }
}

void RobotMeshModel::setCameraInfo(sensor_msgs::CameraInfoConstPtr cam_info){
  cam_info_ = cam_info;
}


void RobotMeshModel::setCamera(){

	//calc field of view for OpenGL Camera;

    double FulstrumWidth, FulstrumHeight, left, right, top, bottom;

    double near_clip = 0.1;
    double farclip = 100;


    double cx= cam_info_->P[2];
    double cy = cam_info_->P[6];

    double fx = cam_info_->P[0];
    double fy = cam_info_->P[5];


    FulstrumWidth = near_clip * cam_info_->width / fx;
    FulstrumHeight = near_clip * cam_info_->height / fy;


    left = FulstrumWidth * (- cx / cam_info_->width);
    right = FulstrumWidth * ( 1.0 - cx / cam_info_->width);
    top = FulstrumHeight * ( cy / cam_info_->height);
    bottom = FulstrumHeight * ( -1.0 + cy / cam_info_->height);


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(left,right, bottom, top, near_clip, farclip);
    glMatrixMode(GL_MODELVIEW);

}


bool RobotMeshModel::setCameraPose(){


	//set camera pose relative to robot links
	if (!tf_.waitForTransform(modelframe_, cameraframe_, current_time_stamp_, ros::Duration(0.5))){
			ROS_ERROR("setting cam pose: could not get transform from %s to %s", modelframe_.c_str(),cameraframe_.c_str() );
			return false;
	 }
	tf_.lookupTransform(modelframe_, cameraframe_, current_time_stamp_, cameraPose);


	//get offset for stereo
	double tx = -1.0 * (cam_info_->P[3] / cam_info_->P[0]);
	tf::Vector3 origin = cameraPose.getOrigin() + cameraPose.getBasis() * tf::Vector3(tx, 0.0, 0.0);

	tf::Vector3 look_at_position = origin + cameraPose.getBasis().getColumn(2);
	 glLoadIdentity();
	 tf::Vector3 up = -cameraPose.getBasis().getColumn(1);

	 gluLookAt(origin.getX() , origin.getY() , origin.getZ() ,
			 look_at_position.getX() , look_at_position.getY(), look_at_position.getZ(),
			 up.getX(), up.getY(), up.getZ());
	 return true;
}


