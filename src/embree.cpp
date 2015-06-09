#include "embree.hpp"

void rtc::init()
{
  /* initialize ray tracing core */
  rtcInit(NULL);
}

void rtc::create_scene(moab::EntityHandle vol)
{
  /* create scene */
  dag_vol_map[vol] = rtcNewScene(RTC_SCENE_ROBUST,RTC_INTERSECT1);
}

void rtc::commit_scene(moab::EntityHandle vol)
{
  /* commit the scene */
  rtcCommit (dag_vol_map[vol]);
}
 
void rtc::shutdown()
{
  /* delete the scene */
  rtcDeleteScene(g_scene);

  /* done with ray tracing */
  rtcExit();
}

/* adds moab range to triangles to the ray tracer */
void rtc::add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense)
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
  unsigned int mesh = rtcNewTriangleMesh(dag_vol_map[vol],RTC_GEOMETRY_STATIC,num_tris,num_verts);

  // now make vertex storage 
  Vertex* vertices = (Vertex*) rtcMapBuffer(dag_vol_map[vol],mesh,RTC_VERTEX_BUFFER);
  
  // need to map moab eh to a vertex id 
  std::map<moab::EntityHandle,int> vert_index_map;
  
  moab::Range::iterator vert_it;
  //  double x_coord,y_coord,z_coord;
  double coords[3];

  //std::cout << "adding " << vert_eh.size() << " vertices to Embree" << std::endl;
  // convert the vertices to embree's format 
  double *coordinates = new double[3*vert_eh.size()];
  rval = MBI->get_coords(vert_eh,coordinates);

  int index;
  for ( vert_it = vert_eh.begin() ; vert_it != vert_eh.end() ; vert_it++ )
    {
      //      index = std::distance(vert_it,vert_eh.begin());
      index = vert_it - vert_eh.begin();
      /*
      std::cout << "Adding vert..." << std::endl;

      std::cout << "MOAB coordinates:" << std::endl;

      std::cout << "x: " << coordinates[index*3] << std::endl
		<< "y: " << coordinates[(index*3)+1] << std::endl
		<< "z: " << coordinates[(index*3)+2] << std::endl;
      */

      // NOTE Embree does not do doubles! 
      vertices[index].x= static_cast<float>(coordinates[index*3]);
      vertices[index].y= static_cast<float>(coordinates[(index*3)+1]);
      vertices[index].z= static_cast<float>(coordinates[(index*3)+2]);    
      
      vert_index_map[*vert_it]=index;

    }
  delete[] coordinates;

  // clear vertex buffer 
  rtcUnmapBuffer(dag_vol_map[vol],mesh,RTC_VERTEX_BUFFER);
  
  // make triangle buffer 
  Triangle* triangles = (Triangle*) rtcMapBuffer(dag_vol_map[vol],mesh,RTC_INDEX_BUFFER);

  moab::Range::iterator tri_it;
  int triangle_idx;
  //std::cout << "adding " << triangles_eh.size() << " triangles to Embree" << std::endl;
  // loop over the triangles and set the mesh connectivity 
  for ( tri_it = triangles_eh.begin() ; tri_it != triangles_eh.end() ; ++tri_it )
    {
      std::vector<moab::EntityHandle> verts;
      std::vector<moab::EntityHandle>::iterator it;
      rval = MBI->get_connectivity(&(*tri_it),1,verts);
      // much much faster than std::distance
      triangle_idx = tri_it - triangles_eh.begin();

      //      triangle_idx = std::distance(tri_it,triangles_eh.end());
      it = verts.begin();

      triangles[triangle_idx].v0 = vert_index_map[*it] ; 
      ++it;
      triangles[triangle_idx].v2 = vert_index_map[*it] ; 
      ++it;
      triangles[triangle_idx].v1 = vert_index_map[*it] ;

      // moab::CartVect v0( vertices[triangles[triangle_idx].v0].x, vertices[triangles[triangle_idx].v0].y, vertices[triangles[triangle_idx].v0].z);
      // moab::CartVect v1( vertices[triangles[triangle_idx].v1].x, vertices[triangles[triangle_idx].v1].y, vertices[triangles[triangle_idx].v1].z);
      // moab::CartVect v2( vertices[triangles[triangle_idx].v2].x, vertices[triangles[triangle_idx].v2].y, vertices[triangles[triangle_idx].v2].z);
      
      
      // moab::CartVect normal = (v1-v0) * (v2-v0);
      // normal.normalize();
      // std::cout << "Triangle normal given to Embree: " <<  normal << std::endl;

    }
  
  // clear triangle buffer 
  rtcUnmapBuffer(dag_vol_map[vol],mesh,RTC_INDEX_BUFFER);
  //  exit(1);
}

bool rtc::point_in_vol(float coordinate[3], float dir[3])
{
  std::vector<int> surfaces;
  std::vector<float> hits;
  get_all_intersections(coordinate,dir,surfaces,hits);
  int num_hits = hits.size();
  dir[0]=-1.0*dir[0],dir[1]=-1.0*dir[1],dir[2]=-1.0*dir[2];
  get_all_intersections(coordinate,dir,surfaces,hits);
  if ( hits.size() == num_hits )
    return true;

  return false;
}

void rtc::ray_fire(moab::EntityHandle volume, float origin[3], float dir[3], float tnear, int &em_surf, float &dist_to_hit, std::vector<float> &norm)
{
  RTCRay ray;
  //  ray.org = origin;
  memcpy(ray.org,origin,3*sizeof(float));
  memcpy(ray.dir,dir,3*sizeof(float));
  //  ray.dir = dir;
  ray.tnear = tnear;
  ray.tfar = 1.0e38;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* fire the ray */
  rtcIntersect(dag_vol_map[volume],ray);

  em_surf = ray.geomID;
  dist_to_hit = ray.tfar;
  norm.clear(); norm.resize(3);

  norm[0] = ray.Ng[0];
  norm[1] = ray.Ng[1];
  norm[2] = ray.Ng[2];
  
  // std::cout << "Hit Surface " << ray.geomID << " after "				    
  // 	    << ray.tfar << " units." << std::endl;					    
  											    
  // std::cout << "Strike position: " << std::endl;					    
  // std::cout << ray.org[0]+(ray.dir[0]*ray.tfar) << " " 				    
  // 	    << ray.org[1]+(ray.dir[1]*ray.tfar) << " " 					    
  // 	    << ray.org[2]+(ray.dir[2]*ray.tfar) << std::endl;				    
   											    
  // std::cout <<  "Ray Origin: " << std::endl;						    
  // std::cout << ray.org[0] << " " << ray.org[1] << " " << ray.org[2] << std::endl;	    
  // std::cout << "Ray Direction: " << std::endl;					    
  // std::cout << ray.dir[0] << " " << ray.dir[1] << " " << ray.dir[2] << std::endl;	    
  // std::cout << ray.tnear << " " << ray.tfar << std::endl;				    
  // std::cout << "Triangle Normal Returned: " << std::endl;				    
  // std::cout << ray.Ng[0] << " " << ray.Ng[1] << " " << ray.Ng[2] << std::endl;	    
  // std::cout << ray.geomID << " " << ray.primID << std::endl;				    
  
}

void rtc::get_all_intersections(float origin[3], float dir[3], std::vector<int> &surfaces,
			       std::vector<float> &distances)
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

    // accrue the hits
    surfaces.push_back(ray.geomID);
    distances.push_back(ray.tfar);

    ray.tnear = 1.00001*ray.tfar;
    ray.tfar = 1.0e38;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
  }
  return;
}


void rtc::psuedo_ris( moab::EntityHandle vol, 
		      std::vector<double> &distances_out, 
		      std::vector<int> &surfs_out, 
		      std::vector<std::array<double, 3> > &tri_norms_out, 
		      const double ray_origin[3], 
		      const double unit_ray_dir[3], 
		      double nonneg_ray_len, 
		      double neg_ray_len)
{

  //get the scene we want to fire on
  RTCScene this_scene = dag_vol_map[vol];
  
  //clear the given vectors and set to the correct size
  distances_out.clear(); distances_out.resize(2);
  surfs_out.clear(); surfs_out.resize(2);
  tri_norms_out.clear(); tri_norms_out.resize(2);

  //convert ray_origin from double to float
  float origin[3], dir[3];
  std::copy( ray_origin, ray_origin+3, origin );
  std::copy( unit_ray_dir, unit_ray_dir+3, dir );

  RTCRay ray;

  memcpy(ray.org,origin,3*sizeof(float));
  memcpy(ray.dir,dir,3*sizeof(float));

  //fire ray in positive direction
  ray.tnear = 0.0f;
  ray.tfar = float(nonneg_ray_len);
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* fire the ray */
  rtcIntersect(this_scene,ray);

  distances_out[1] = ray.tfar;
  surfs_out[1] = ray.geomID;
  tri_norms_out[1][0] = double(ray.Ng[0]);
  tri_norms_out[1][1] = double(ray.Ng[1]);
  tri_norms_out[1][2] = double(ray.Ng[2]);

  // now fire in the negative direction
  ray.dir[0] *= -1; ray.dir[1] *= -1; ray.dir[2] *= -1; 

  ray.tfar = float(neg_ray_len);

  ray.tnear = 0.0f;

  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* fire the ray */
  rtcIntersect(this_scene,ray);

  distances_out[0] = ray.tfar;
  surfs_out[0] = ray.geomID;
  tri_norms_out[0][0] = double(ray.Ng[0]);
  tri_norms_out[0][1] = double(ray.Ng[1]);
  tri_norms_out[0][2] = double(ray.Ng[2]);





  return;
}
