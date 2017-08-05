#include <ros/ros.h>

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


int main(int argc, char **argv)
{
  Assimp::Importer as_importer;
  as_importer.SetIOHandler(new ResourceIOSystem());
  std::string filename;
  if (argc == 2)
    filename = argv[1];
  else
    filename = "package://pr2_description/meshes/base_v0/base.stl";
  const aiScene* scene =
    as_importer.ReadFile(filename.c_str(), aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
  
  if (!scene)
    {
      ROS_ERROR("Could not load resource [%s]: %s", filename.c_str(),
                as_importer.GetErrorString());
    }
}
