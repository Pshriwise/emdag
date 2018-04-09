#include "embree.hpp"
#include <assert.h>

  void test_func(Vec3da v);

void DblTriBounds(void* tris_i, size_t item, RTCBounds& bounds_o) {

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];  

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  assert(conn.size() == 3);
  moab::CartVect coords[3];
  rval = mbi->get_coords(&conn[0], 1, coords[0].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, coords[1].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, coords[2].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  double bump_val = 5e-03;
  
  bounds_o.lower_x = std::min(coords[0][0],std::min(coords[1][0],coords[2][0]));
  bounds_o.lower_y = std::min(coords[0][1],std::min(coords[1][1],coords[2][1]));
  bounds_o.lower_z = std::min(coords[0][2],std::min(coords[1][2],coords[2][2]));

  bounds_o.upper_x = std::max(coords[0][0],std::max(coords[1][0],coords[2][0]));
  bounds_o.upper_y = std::max(coords[0][1],std::max(coords[1][1],coords[2][1]));
  bounds_o.upper_z = std::max(coords[0][2],std::max(coords[1][2],coords[2][2]));
  
  bounds_o.lower_x -= bump_val; bounds_o.lower_y -= bump_val; bounds_o.lower_z -= bump_val; 
  bounds_o.upper_x += bump_val; bounds_o.upper_y += bump_val; bounds_o.upper_z += bump_val; 

  return;
}


void DblTriIntersectFunc(void* tris_i, RTCDRay& ray, size_t item) {

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];  

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  Vec3da coords[3];
  rval = mbi->get_coords(&conn[0], 1, &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, &(coords[1][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, &(coords[2][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  
  double dist;
  double nonneg_ray_len = 1e17;
  const double* ptr = &nonneg_ray_len;
  Vec3da ray_org(ray.dorg);
  Vec3da ray_dir(ray.ddir);
  
  bool hit = plucker_ray_tri_intersect(coords, ray_org, ray_dir, dist, ptr);

  if ( true ) {
    ray.dtfar = 10.0;
    ray.tfar = 10.0;
    ray.u = 0.0f;
    ray.v = 0.0f;
    ray.geomID = this_tri.geomID;
    ray.primID = (unsigned int) item;

    Vec3da normal = cross((coords[1] - coords[0]),(coords[2] - coords[0]));

    ray.Ng[0] = normal[0];
    ray.Ng[1] = normal[1];
    ray.Ng[2] = normal[2];
  }

  return;
}

void DblTriOccludedFunc(void* tris_i, RTCDRay& ray, size_t item) {

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];  
  RTCBounds bounds;

  moab::Interface* mbi = (moab::Interface*) this_tri.moab_instance;
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&(this_tri.handle), 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  moab::CartVect coords[3];
  rval = mbi->get_coords(&conn.front(), conn.size(), coords[0].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

  double dist;
  double nonneg_ray_len = 1e37;
  double* ptr = &nonneg_ray_len;
  moab::CartVect ray_org(ray.dorg);
  moab::CartVect ray_dir(ray.ddir);
  
  bool hit = moab::GeomUtil::plucker_ray_tri_intersect(coords, ray_org, ray_dir, 0.0, dist, ptr);
  if ( hit ) {
    ray.geomID = 0;
  }
}





void rtc::init()
{
  /* initialize ray tracing core */
  rtcInit(NULL);
}


double dot_prod( RTCRay &ray )
{

  float result = ray.dir[0]*ray.Ng[0];
  result += ray.dir[1]*ray.Ng[1];
  result += ray.dir[2]*ray.Ng[2]; 
  
  return result;

}



void intersectionFilter(void* ptr, RTCRay2 &ray) 
{

  switch(ray.rf_type) 
    {
    case 0: //if this is a typical ray_fire, check the dot_product
      if ( 0 > dot_prod(ray) )
	ray.geomID = RTC_INVALID_GEOMETRY_ID;
      break;
    case 1: //if this is a point_in_vol fire, do nothing
      break;
    }

}

void rtc::set_offset(moab::Range &vols) {

  sceneOffset = *vols.begin();
  std::cout << "Scene offset: " << sceneOffset << std::endl;
  scenes.resize(vols.back()-sceneOffset+1);
  std::cout << "Size of scenes: " << scenes.size() << std::endl;
  
}

void rtc::create_scene(moab::EntityHandle vol)
{
  /* create scene */
  scenes[vol-sceneOffset] = rtcNewScene(RTC_SCENE_ROBUST,RTC_INTERSECT1);
}

void rtc::commit_scene(moab::EntityHandle vol)
{
  /* commit the scene */
  rtcCommit(scenes[vol-sceneOffset]);
}
 
void rtc::shutdown()
{
  /* delete the scene */
  rtcDeleteScene(g_scene);

  /* done with ray tracing */
  rtcExit();
}

void rtc::create_vertex_map(moab::Interface* MBI)
{
  
  std::vector<moab::EntityHandle> all_verts;
  //use the moab interface to get all vertices in the mesh 
  moab::ErrorCode rval = MBI->get_entities_by_type(0, moab::MBVERTEX, all_verts, true);
  if (moab::MB_SUCCESS != rval ) 
    std::cout << "Error getting the mesh vertices for the global map." << std::endl;

  //now create a structure with enough room for all verts in the mesh
  int num_verts = all_verts.size();

  vertices.resize(num_verts);

  //now populate the structure
  std::vector<moab::EntityHandle>::iterator vert_it;

  double coords[3];

  //std::cout << "adding " << vert_eh.size() << " vertices to Embree" << std::endl;
  // convert the vertices to embree's format 
  double *coordinates = new double[3*all_verts.size()];
  rval = MBI->get_coords(&(all_verts[0]), (int)all_verts.size(),coordinates);

  int index;
  for ( vert_it = all_verts.begin() ; vert_it != all_verts.end() ; vert_it++ )
    {

      index = vert_it - all_verts.begin();
      
      // std::cout << "Adding vert..." << std::endl;

      // std::cout << "MOAB coordinates:" << std::endl;

      // std::cout << "x: " << coordinates[index*3] << std::endl
      // 		<< "y: " << coordinates[(index*3)+1] << std::endl
      // 		<< "z: " << coordinates[(index*3)+2] << std::endl;

      // std::cout << "This vertex's index is: " << index << std::endl;

      // NOTE Embree does not do doubles! 
      vertices[index].x= static_cast<float>(coordinates[index*3]);
      vertices[index].y= static_cast<float>(coordinates[(index*3)+1]);
      vertices[index].z= static_cast<float>(coordinates[(index*3)+2]);    
      
      global_vertex_map.insert(std::pair<moab::EntityHandle,int>(*vert_it,index));

    }
  delete[] coordinates;

  //now set the buffer pointer and size
  vertex_buffer_ptr = (void*) &(vertices[0]);
  vertex_buffer_size = num_verts;
  
}

DblTri* rtc::add_Dtriangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense) {

  size_t num_tris = triangles_eh.size();
  RTCScene scene = scenes[vol-sceneOffset];
  
  unsigned int tri_geom = rtcNewUserGeometry(scene, num_tris);

  DblTri* tris = (DblTri*) malloc(num_tris*sizeof(DblTri));
  rtcSetUserData(scene,tri_geom,tris);
  for(size_t i = 0; i < num_tris; i++) {
    tris[i].moab_instance = (void*)MBI;
    tris[i].handle = triangles_eh[i];
    tris[i].geomID = tri_geom;
  }
  
  rtcSetIntersectionFilterFunction(scene, tri_geom, (RTCFilterFunc)&intersectionFilter);
  rtcSetBoundsFunction(scene, tri_geom, (RTCBoundsFunc)&DblTriBounds);  
  rtcSetIntersectFunction(scene, tri_geom, (RTCIntersectFunc)&DblTriIntersectFunc);
  rtcSetOccludedFunction(scene, tri_geom, (RTCOccludedFunc)&DblTriOccludedFunc);

  return tris;
}

/* adds moab range to triangles to the ray tracer */
void rtc::add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense)
{
  moab::ErrorCode rval;

  int num_tris = triangles_eh.size();

  /* make the mesh */
  unsigned int mesh = rtcNewTriangleMesh(scenes[vol-sceneOffset],RTC_GEOMETRY_STATIC,num_tris,vertex_buffer_size);

  //set the intersection filter function 
  rtcSetIntersectionFilterFunction(scenes[vol-sceneOffset], mesh, (RTCFilterFunc)&intersectionFilter);

  // now set the vertex storage 
  rtcSetBuffer(scenes[vol-sceneOffset],mesh,RTC_VERTEX_BUFFER, vertex_buffer_ptr, 0, sizeof(Vertex));
    
  // make triangle buffer 
  Triangle* triangles = (Triangle*) rtcMapBuffer(scenes[vol-sceneOffset],mesh,RTC_INDEX_BUFFER);

  moab::Range::iterator tri_it;
  int triangle_idx;

  // loop over the triangles and set the mesh connectivity 
  for ( tri_it = triangles_eh.begin() ; tri_it != triangles_eh.end() ; ++tri_it )
    {
      std::vector<moab::EntityHandle> verts;
      std::vector<moab::EntityHandle>::iterator it;
      //get the vertex handle connectivity of this triangle
      rval = MBI->get_connectivity(&(*tri_it),1,verts);

      //set the index of the current triangle based on its position
      triangle_idx = tri_it - triangles_eh.begin();

      //set the vertex iterator
      it = verts.begin();

      //adjust triangle normals for the surface to volume sense
      if ( 1 == sense )
	{
	  triangles[triangle_idx].v0 = global_vertex_map[*it] ; 
	  ++it;
	  triangles[triangle_idx].v2 = global_vertex_map[*it] ; 
	  ++it;
	  triangles[triangle_idx].v1 = global_vertex_map[*it] ;
	}
      else if ( -1 == sense )
	{
	  triangles[triangle_idx].v0 = global_vertex_map[*it] ; 
	  ++it;
	  triangles[triangle_idx].v1 = global_vertex_map[*it] ; 
	  ++it;
	  triangles[triangle_idx].v2 = global_vertex_map[*it] ;
	}

      
       // moab::CartVect v0( vert_buff_ptr[triangles[triangle_idx].v0].x, vert_buff_ptr[triangles[triangle_idx].v0].y, vert_buff_ptr[triangles[triangle_idx].v0].z);
       // moab::CartVect v1( vert_buff_ptr[triangles[triangle_idx].v1].x, vert_buff_ptr[triangles[triangle_idx].v1].y, vert_buff_ptr[triangles[triangle_idx].v1].z);
       // moab::CartVect v2( vert_buff_ptr[triangles[triangle_idx].v2].x, vert_buff_ptr[triangles[triangle_idx].v2].y, vert_buff_ptr[triangles[triangle_idx].v2].z);
      
       // std::cout << "New triangle added: " << std::endl << v0 << std::endl << v1 << std::endl << v2 << std::endl;
      
      // moab::CartVect normal = (v1-v0) * (v2-v0);
      // normal.normalize();
      // std::cout << "Triangle normal given to Embree: " <<  normal << std::endl;

    }
  

  //unmap triangle and vertex buffers 
  rtcUnmapBuffer(scenes[vol-sceneOffset],mesh,RTC_INDEX_BUFFER);

  rtcUnmapBuffer(scenes[vol-sceneOffset],mesh,RTC_VERTEX_BUFFER);

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

void rtc::ray_fire(moab::EntityHandle volume, const double origin[3], const double direction[3], rf_type filt_func, double tnear, int &em_surf, double &dist_to_hit, float norm[3])
{

  RTCDRay ray;

  ray.dorg[0] = origin[0]; ray.dorg[1] = origin[1]; ray.dorg[2] = origin[2];
  ray.ddir[0] = direction[0]; ray.ddir[1] = direction[1]; ray.ddir[2] = direction[2];

  //populate the ray structure with the incoming/default information as needed
  ray.org[0] = origin[0]; ray.org[1] = origin[1]; ray.org[2] = origin[2];
  ray.dir[0] = direction[0]; ray.dir[1] = direction[1]; ray.dir[2] = direction[2];

  ray.tnear = tnear;
  ray.tfar = 1.0e38;
  ray.dtfar = 1.0e38;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  ray.rf_type = (int)filt_func;

  /* fire the ray */
  rtcIntersect(scenes[volume-sceneOffset],*((RTCRay*)&ray));

  //get the critical information from the ray
  em_surf = ray.geomID;
  dist_to_hit = ray.dtfar;
  norm[0] = ray.Ng[0];
  norm[1] = ray.Ng[1];
  norm[2] = ray.Ng[2];
 
  //if we don't hit a surface, check right behind the ray to see if we're ahead of a surface
  // (do this only for queries comeing from DagMC::ray_fire)
  if (RTC_INVALID_GEOMETRY_ID == ray.geomID && rf_type::RF == filt_func) 
    { 

      //turn the ray around 
      ray.dir[0] *= -1; 
      ray.dir[1] *= -1; 
      ray.dir[2] *= -1; 
      //set the distance to some small tolerance (1e-4) 
      ray.tfar = 1.0e-3;
      ray.rf_type = rf_type::PIV; //to allow for hits against the normal
      /* fire the ray */
      rtcIntersect(scenes[volume-sceneOffset],*((RTCRay*)&ray));

      //if we get a hit, return that surface ID and a distance of zero.
      if( RTC_INVALID_GEOMETRY_ID != ray.geomID) 

  	{ 
	  
  	  em_surf = ray.geomID;
  	  dist_to_hit = 0;
	  norm[0] = 0;
	  norm[1] = 0;
	  norm[2] = 0;
	  
  	  norm[0] = ray.Ng[0];
  	  norm[1] = ray.Ng[1];
  	  norm[2] = ray.Ng[2];
	  
  	}

    }

  // std::cout << "Ray's Barycentric coords: u= " << ray.u << " v= "
  // 	    << ray.v << " w = " << 1-ray.u-ray.v << std::endl;
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
  RTCScene this_scene = scenes[vol-sceneOffset];
  
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

  
