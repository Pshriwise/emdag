#include "embree.hpp"

void rtc::init()
{
  /* initialize ray tracing core */
  rtcInit(NULL);
}

void rtc::create_scene()
{
  /* create scene */
  g_scene = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT1);
}

void rtc::commit_scene()
{
  /* commit the scene */
  rtcCommit (g_scene);
}
 
void rtc::shutdown()
{
  /* delete the scene */
  rtcDeleteScene(g_scene);

  /* done with ray tracing */
  rtcExit();
}

/* adds moab range to triangles to the ray tracer */
void rtc::add_volume(moab::Interface* MBI, moab::Range triangles_eh)
{
  moab::ErrorCode rval;
  moab::Range vert_eh;
  /* get the vertices - get_vertices doesnt work ?*/
  rval = MBI->get_adjacencies(triangles_eh,0,true,vert_eh,moab::Interface::UNION);
  //  rval = MBI->get_vertices(triangles_eh,vert_eh);

  int num_tris = triangles_eh.size();
  int num_verts = vert_eh.size();

  // std::cout << "Importing " << num_tris << " triangles" << std::endl;
  // std::cout << "Importing " << num_verts << " vertices" << std::endl;


  /* make the mesh */
  unsigned int mesh = rtcNewTriangleMesh(g_scene,RTC_GEOMETRY_STATIC,num_tris,num_verts);


  // now make vertex storage 
  Vertex* vertices = (Vertex*) rtcMapBuffer(g_scene,mesh,RTC_VERTEX_BUFFER);
  
  // need to map moab eh to a vertex id 
  std::map<moab::EntityHandle,int> vert_index_map;
  
  moab::Range::iterator vert_it;
  //  double x_coord,y_coord,z_coord;
  double coords[3];

  std::cout << "adding " << vert_eh.size() << " vertices to Embree" << std::endl;
  // convert the vertices to embree's format 
  double *coordinates = new double[3*vert_eh.size()];
  rval = MBI->get_coords(vert_eh,coordinates);

  int index;
  for ( vert_it = vert_eh.begin() ; vert_it != vert_eh.end() ; ++vert_it )
    {
      //      index = std::distance(vert_it,vert_eh.begin());
      index = vert_it - vert_eh.begin();
      // NOTE Embree does not do doubles! 
      vertices[index].x= static_cast<float>(coordinates[index*3]);
      vertices[index].y= static_cast<float>(coordinates[(index*3)+1]);
      vertices[index].z= static_cast<float>(coordinates[(index*3)+2]);    
      
      vert_index_map[*vert_it]=index;

    }
  delete[] coordinates;

  // clear vertex buffer 
  rtcUnmapBuffer(g_scene,mesh,RTC_VERTEX_BUFFER);
  
  // make triangle buffer 
  Triangle* triangles = (Triangle*) rtcMapBuffer(g_scene,mesh,RTC_INDEX_BUFFER);

  moab::Range::iterator tri_it;
  int triangle_idx;
  std::cout << "adding " << triangles_eh.size() << " triangles to Embree" << std::endl;
  // loop over the triangles and set the mesh connectivity 
  for ( tri_it = triangles_eh.begin() ; tri_it != triangles_eh.end() ; ++tri_it )
    {
      moab::Range verts;
      moab::Range::iterator it;
      rval = MBI->get_adjacencies(&(*tri_it),1,0,false,verts,moab::Interface::UNION);
      // much much faster than std::distance
      triangle_idx = tri_it - triangles_eh.begin();

      //      triangle_idx = std::distance(tri_it,triangles_eh.end());
      it = verts.begin();

      triangles[triangle_idx].v0 = vert_index_map[*it] ; ++it;
      triangles[triangle_idx].v1 = vert_index_map[*it] ; ++it;
      triangles[triangle_idx].v2 = vert_index_map[*it] ;
    }
  
  // clear triangle buffer 
  rtcUnmapBuffer(g_scene,mesh,RTC_INDEX_BUFFER);
  //  exit(1);
}

void rtc::point_in_vol(float coordinate[3], float dir[3], int &region_id)
{
  return;
}

void rtc::ray_fire(float origin[3], float dir[3])
{
  RTCRay ray;
  //  ray.org = origin;
  memcpy(ray.org,origin,3*sizeof(float));
  memcpy(ray.dir,dir,3*sizeof(float));
  //  ray.dir = dir;
  ray.tnear = 0.0f;
  ray.tfar = 1.0e38;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* fire the ray */
  rtcIntersect(g_scene,ray);

  std::cout << ray.org[0]+(ray.dir[0]*ray.tfar) << " " 
	    << ray.org[1]+(ray.dir[1]*ray.tfar) << " " 
	    << ray.org[2]+(ray.dir[2]*ray.tfar) << std::endl;

  /*  
  std::cout << ray.org[0] << " " << ray.org[1] << " " << ray.org[2] << std::endl;
  std::cout << ray.dir[0] << " " << ray.dir[1] << " " << ray.dir[2] << std::endl;
  std::cout << ray.tnear << " " << ray.tfar << std::endl;
  std::cout << ray.geomID << " " << ray.primID << std::endl;
  */
}

void rtc::get_all_intersections(float origin[3], float dir[3])
{
  RTCRay ray;
  //  ray.org = origin;
  memcpy(ray.org,origin,3*sizeof(float));
  memcpy(ray.dir,dir,3*sizeof(float));
  //  ray.dir = dir;
  ray.tnear = 0.0f;
  ray.tfar = 1.0e38;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  
  while(true) {
    /* fire the ray */
    rtcIntersect(g_scene,ray);

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID )
      break;

    std::cout << ray.geomID << " " << ray.tfar << std::endl;
    ray.tnear = 1.00001*ray.tfar;
    ray.tfar = 1.0e38;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
  }
  return;
}

