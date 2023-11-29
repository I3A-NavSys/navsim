#include <gazebo/gazebo.hh>

namespace gazebo
{
  class HelloWorld : public WorldPlugin
  {
    public: HelloWorld() : WorldPlugin()
    {
      printf("Hello World!\n");
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Lógica de inicialización de tu plugin
      gzmsg << "¡Hola desde mi plugin de mundo!" << std::endl;
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(HelloWorld)
}


