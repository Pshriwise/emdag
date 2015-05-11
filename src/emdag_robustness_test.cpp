#include <iostream>
#include "MBTagConventions.hpp"
#include "moab/CartVect.hpp"
#include "moab/Range.hpp"
#include "moab/Core.hpp"
#include "moab/GeomUtil.hpp"
#include "moab/FileOptions.hpp"
#include <ctime> // for timing

#include "embree.hpp"

moab::Interface* MBI(); 

int load_file(char* filename); // load the current h5m file
int get_triangles_on_volume(moab::EntityHandle volume, moab::Range &triangles); // get the triangles from the first volume
int get_triangles_on_surface(moab::EntityHandle surface, moab::Range &triangles); // get the triangles from the first volume

int get_all_volumes(moab::Range &volumes); // get the entity handles of all the volumes in the problem
int get_all_surfaces(moab::Range &surfaces); // get the entity handles of all the volumes in the problem

void iso_dir(float direction[3],int seed);
float rand(int seed);

/* main driver */
int main(int argc, char *argv[])
{
  // load the moab_file
  std::cout << "loading file, " << argv[1] << "..." << std::endl;
  int errorcode = load_file(argv[1]);
  if(errorcode != 0)
    {
      std::cout << "Failure" << std::endl;
      return errorcode;
    }

  std::cout << "loading complete." << std::endl;

  // extract the volumes
  moab::Range entities;
  moab::Range::iterator it;
  //  errorcode = get_all_volumes(volumes);
  errorcode = get_all_surfaces(entities);

  rtc *RTC = new rtc;

  RTC->init();
  RTC->create_scene();
  moab::Range triangles;
  for ( it = entities.begin() ; it != entities.end() ; ++it )
    {
      // get all the triangles associated with a given volume
      errorcode = get_triangles_on_surface(*it,triangles);
      RTC->add_triangles(MBI(),triangles);
    }

  RTC->commit_scene();
  
  float pos[3] = {0.,0.,0.};
  float dir[3]; // = {1.,0.,0.};

  int seed = 123456789;
  int stride = 7;
  std::clock_t start;
  double duration;
  double total = 0.0;

  int num_rays = 5000000;
  std::vector<int> surfaces;
  std::vector<float> hits;
  int surface_hit;
  float distance_to_hit;
  int misses = 0;
  int rays_fired = 0;
  std::cout << "Firing rays..." << std::endl;
  moab::ErrorCode rval;
  for ( int i = 0 ; i < triangles.size(); i++ )
    {
      //iso_dir(dir,seed+(i*stride));
      //      std::cout << i << " ";
      //      RTC->get_all_intersections(pos,dir,surfaces,hits);
      //      return 0;

      //get the traingle
      moab::EntityHandle this_tri = triangles[i];

      //get the vertices of the triangle
      moab::Range verts;
      rval = MBI()->get_adjacencies(&this_tri,1,0,true,verts);
      if(rval != moab::MB_SUCCESS || verts.size() != 3)
	{
	  std::cout << "Error getting triangle verts." << std::endl;
	  std::cout << "Verts found " << verts.size() << std::endl;
	  return 1;
	}

      //get the coordinates of the vertices
      double xs[3], ys[3], zs[3];

      rval = MBI()->get_coords(verts, xs, ys, zs);
      if(rval != moab::MB_SUCCESS)
	{
	  std::cout << "Error getting vertex coordinates." << std::endl;
	  return 1;
	}

      moab::CartVect v0(xs[0],ys[0],zs[0]);
      moab::CartVect v1(xs[1],ys[1],zs[1]);
      moab::CartVect v2(xs[2],ys[2],zs[2]);
      double third = 0.3333333333333333333333333;
      //now prepare the different directions for firing...
      std::vector<moab::CartVect> dirs;

      //center of triangle
      dirs.push_back(third*v0+third*v2+third*v2);
      //middle of edge 0
      dirs.push_back(unit(0.5*v0+0.5*v1));
      //middle of edge 1
      dirs.push_back(unit(0.5*v1+0.5*v2));
      //middle of edge 2
      dirs.push_back(unit(0.5*v2+0.5*v0));

      //at vert 0
      dirs.push_back(unit(v0));
      //at vert 1
      dirs.push_back(unit(v1));
      //at vert 2
      dirs.push_back(unit(v2));

      for(unsigned int j = 0; j < dirs.size(); j++)
	{
	  float this_dir[3], tri_norm[3];
	  dirs[j].get(this_dir);
	  start = std::clock();      
	  RTC->ray_fire(pos,this_dir,surface_hit,distance_to_hit, tri_norm);
	  duration = (std::clock() - start)/ (double) CLOCKS_PER_SEC;
	  if (-1 == surface_hit) misses++;
	  total += duration;
	  rays_fired++;
	}
    }

  std::cout << rays_fired << " took " << total << " seconds, time per ray " << total/double(rays_fired) << std::endl;
  std::cout << "Missed rays: " << misses << std::endl;

  RTC->shutdown();

  return 0;
}

moab::Interface* MBI()
{
  static moab::Core instance;
  return &instance;
}


/* load the MOAB file */
int load_file(char* filename)
{
  moab::ErrorCode rval; // reusable error state
  moab::EntityHandle file_set;

  // create entityset for file contents
  rval = MBI()->create_meshset(moab::MESHSET_SET,file_set);
  // put file contents in entity set
  rval = MBI()->load_file(filename,&file_set);

  return rval;
}

/* get all the volumes */
int get_all_volumes(moab::Range &volumes)
{
    // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  int three[1] = {3};
  const void* const dim[1] = {three};
  moab::Tag geom_tag;

  // get the tag handle
  rval = MBI()->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = MBI()->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,volumes);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get volumes from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << volumes.size() << " volumes" << std::endl;
    }

  return rval;
}

/* get all the volumes */
int get_all_surfaces(moab::Range &surfaces)
{
    // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  int two[1] = {2};
  const void* const dim[1] = {two};
  moab::Tag geom_tag;

  // get the tag handle
  rval = MBI()->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = MBI()->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,surfaces);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get surfaces from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << surfaces.size() << " surfaces" << std::endl;
    }

  return rval;
}

/* get the triangles for the given surface */
int get_triangles_on_surface(moab::EntityHandle surface, moab::Range &triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = MBI()->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = MBI()->tag_get_data(id_tag,&(surface),1,&id);
  

  rval = MBI()->get_entities_by_type(surface, moab::MBTRI,triangles);
  
  std::cout << "Surface " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}


/* get the triangles for the given volume */
int get_triangles_on_volume(moab::EntityHandle volume, moab::Range &triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = MBI()->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = MBI()->tag_get_data(id_tag,&(volume),1,&id);
  

  moab::Range child_surface_sets;
  // get the child sets are all the surfaces
  rval = MBI()->get_child_meshsets(volume,child_surface_sets);

  moab::Range::iterator surf_it;
  // moab ranges are additive, so it gets appended to every time
  for ( surf_it = child_surface_sets.begin() ; surf_it != child_surface_sets.end() ; ++surf_it)
    {
      rval = MBI()->get_entities_by_type(*surf_it,moab::MBTRI,triangles);
    }
  std::cout << "Volume " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

void iso_dir(float direction[3],int seed)
{
  float theta,phi; // components

  theta = rand(seed)*2*3.14149;
  phi = rand(seed+1)*3.14149;

  direction[0] = cos(theta)*sin(phi);
  direction[1] = sin(theta)*sin(phi);
  direction[2] = cos(phi);

  return;
}


float rand(int seed)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<> uni(0,1);
  return uni(gen);
}
