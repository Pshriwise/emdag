#include "sys/platform.h"
#include "sys/ref.h"
#include "sys/thread.h"
#include "sys/sysinfo.h"
#include "sys/sync/barrier.h"
#include "sys/sync/mutex.h"
#include "sys/sync/condition.h"
#include "math/vec3.h"
#include "math/bbox.h"
#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"
#include "../kernels/common/default.h"
#include <vector>

#include "moab/Core.hpp"
#include "moab/Range.hpp"

struct Triangle { int v0, v1, v2; };

struct Vertex   { float x,y,z,r; };

class rtc {
  private:
    RTCScene g_scene;
    int8 *cfg;
  public:
    void init();
    void create_scene();
    void commit_scene();
    void finalise_scene();
    void shutdown(); 
    void add_volume(moab::Interface* MBI, moab::Range triangles_eh);
    void ray_fire(float origin[3], float dir[3]);
};

