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

struct Triangle { int v0, v1, v2; };

struct Vertex   { float x,y,z,r; };

class rtc {
  private:
    RTCScene g_scene;
  std::map<moab::EntityHandle,RTCScene> dag_vol_map;
  public:
    void init();
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
    void finalise_scene();
    void shutdown(); 
  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh);
  void ray_fire(float origin[3], float dir[3], int &em_surf, float &dist_to_hit, std::vector<float> &norm);
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

