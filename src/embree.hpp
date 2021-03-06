/*
#include "sys/platform.h"
#include "sys/ref.h"
#include "sys/thread.h"
#include "sys/sysinfo.h"
#include "sys/sync/barrier.h"
#include "sys/sync/mutex.h"
#include "sys/sync/condition.h"
#include "math/vec3.h"
#include "math/bbox.h"
*/
#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"
//#include "../kernels/common/default.h"
#include <array>
#include <vector>
#include <iostream>
#include "moab/Core.hpp"
#include "moab/Range.hpp"
#include "moab/CartVect.hpp"


struct Triangle { int v0, v1, v2; };

struct Vertex   { float x,y,z; };


struct RTCRay2 : RTCRay { int rf_type; };

enum rf_type { RF, PIV};

class rtc {
  private:
    RTCScene g_scene;
  std::map<moab::EntityHandle,RTCScene> dag_vol_map;
  std::map<moab::EntityHandle,int> global_vertex_map;
  std::vector<RTCScene> scenes;
  moab::EntityHandle sceneOffset;
  
  public:
  void *vertex_buffer_ptr;
  int vertex_buffer_size;
  std::vector<Vertex> vertices;
  enum rf_type { RF, PIV };
  void set_offset(moab::Range &vols);
  void init();
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
  void finalise_scene();
  void shutdown(); 
  rf_type ray_fire_type;
  void create_vertex_map(moab::Interface* MBI);
  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense);
  void ray_fire(moab::EntityHandle volume, float origin[3], float dir[3], rf_type filt_func, float tnear,  int &em_surf, float &dist_to_hit, float norm[3]);
  bool point_in_vol(float coordinate[3], float dir[3]);
  void get_all_intersections(float origin[3], float dir[3], std::vector<int> &surfaces,
			     std::vector<float> &distances);

  void psuedo_ris( moab::EntityHandle vol, 
		   std::vector<double> &distances_out, 
		   std::vector<int> &surfs_out, 
		   std::vector<std::array<double,3> > &tri_norms_out, 
		   const double ray_origin[3], 
		   const double unit_ray_dir[3], 
		   double nonneg_ray_len, 
		   double neg_ray_len);


};

