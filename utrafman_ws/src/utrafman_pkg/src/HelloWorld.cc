#include <gazebo/gazebo.hh>

namespace gazebo
{
  class HelloWorld : public WorldPlugin
  {

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      printf("Hello World!  LOAD\n");
      gzmsg << "¡Hola desde mi plugin de mundo!" << std::endl;
    }

    void Init()
    {
        printf("Hello World!  INIT\n");
    }

  };
  GZ_REGISTER_WORLD_PLUGIN(HelloWorld)
}


