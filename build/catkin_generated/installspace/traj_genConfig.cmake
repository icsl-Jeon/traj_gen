# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(traj_gen_CONFIG_INCLUDED)
  return()
endif()
set(traj_gen_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(traj_gen_SOURCE_PREFIX /home/jbs/catkin_ws/src/traj_gen)
  set(traj_gen_DEVEL_PREFIX /home/jbs/catkin_ws/src/traj_gen/build/devel)
  set(traj_gen_INSTALL_PREFIX "")
  set(traj_gen_PREFIX ${traj_gen_DEVEL_PREFIX})
else()
  set(traj_gen_SOURCE_PREFIX "")
  set(traj_gen_DEVEL_PREFIX "")
  set(traj_gen_INSTALL_PREFIX /usr/local)
  set(traj_gen_PREFIX ${traj_gen_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'traj_gen' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(traj_gen_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include;/home/jbs/lib/cgal/SearchStructures/include;/home/jbs/lib/cgal/Surface_sweep_2/include;/home/jbs/lib/cgal/Solver_interface/include;/home/jbs/lib/cgal/Algebraic_kernel_for_circles/include;/home/jbs/lib/cgal/Matrix_search/include;/home/jbs/lib/cgal/Algebraic_foundations/include;/home/jbs/lib/cgal/Triangulation_3/include;/home/jbs/lib/cgal/OpenNL/include;/home/jbs/lib/cgal/Voronoi_diagram_2/include;/home/jbs/lib/cgal/Kernel_23/include;/home/jbs/lib/cgal/Segment_Delaunay_graph_Linf_2/include;/home/jbs/lib/cgal/Periodic_3_mesh_3/include;/home/jbs/lib/cgal/Mesh_2/include;/home/jbs/lib/cgal/Heat_method_3/include;/home/jbs/lib/cgal/Convex_hull_3/include;/home/jbs/lib/cgal/Distance_2/include;/home/jbs/lib/cgal/Polygon_mesh_processing/include;/home/jbs/lib/cgal/Mesher_level/include;/home/jbs/lib/cgal/Polytope_distance_d/include;/home/jbs/lib/cgal/Segment_Delaunay_graph_2/include;/home/jbs/lib/cgal/Cartesian_kernel/include;/home/jbs/lib/cgal/Modular_arithmetic/include;/home/jbs/lib/cgal/Nef_3/include;/home/jbs/lib/cgal/Surface_mesh_deformation/include;/home/jbs/lib/cgal/Scale_space_reconstruction_3/include;/home/jbs/lib/cgal/Inventor/include;/home/jbs/lib/cgal/Stream_lines_2/include;/home/jbs/lib/cgal/Kernel_d/include;/home/jbs/lib/cgal/AABB_tree/include;/home/jbs/lib/cgal/Convex_hull_2/include;/home/jbs/lib/cgal/Poisson_surface_reconstruction_3/include;/home/jbs/lib/cgal/Polyhedron/include;/home/jbs/lib/cgal/Envelope_3/include;/home/jbs/lib/cgal/CGAL_ipelets/include;/home/jbs/lib/cgal/Union_find/include;/home/jbs/lib/cgal/Property_map/include;/home/jbs/lib/cgal/Surface_mesh_skeletonization/include;/home/jbs/lib/cgal/Filtered_kernel/include;/home/jbs/lib/cgal/Minkowski_sum_3/include;/home/jbs/lib/cgal/Polyhedron_IO/include;/home/jbs/lib/cgal/Installation/include;/home/jbs/lib/cgal/Cone_spanners_2/include;/home/jbs/lib/cgal/QP_solver/include;/home/jbs/lib/cgal/Straight_skeleton_2/include;/home/jbs/lib/cgal/Intersections_3/include;/home/jbs/lib/cgal/Generator/include;/home/jbs/lib/cgal/Principal_component_analysis/include;/home/jbs/lib/cgal/Generalized_map/include;/home/jbs/lib/cgal/GraphicsView/include;/home/jbs/lib/cgal/Circular_kernel_2/include;/home/jbs/lib/cgal/Optimal_transportation_reconstruction_2/include;/home/jbs/lib/cgal/CGAL_ImageIO/include;/home/jbs/lib/cgal/Homogeneous_kernel/include;/home/jbs/lib/cgal/Hash_map/include;/home/jbs/lib/cgal/Subdivision_method_3/include;/home/jbs/lib/cgal/Spatial_sorting/include;/home/jbs/lib/cgal/Modifier/include;/home/jbs/lib/cgal/Classification/include;/home/jbs/lib/cgal/Surface_mesh_parameterization/include;/home/jbs/lib/cgal/Algebraic_kernel_d/include;/home/jbs/lib/cgal/Three/include;/home/jbs/lib/cgal/Periodic_2_triangulation_2/include;/home/jbs/lib/cgal/Visibility_2/include;/home/jbs/lib/cgal/Minkowski_sum_2/include;/home/jbs/lib/cgal/Principal_component_analysis_LGPL/include;/home/jbs/lib/cgal/Convex_decomposition_3/include;/home/jbs/lib/cgal/STL_Extension/include;/home/jbs/lib/cgal/Nef_2/include;/home/jbs/lib/cgal/Box_intersection_d/include;/home/jbs/lib/cgal/Mesh_3/include;/home/jbs/lib/cgal/Jet_fitting_3/include;/home/jbs/lib/cgal/Convex_hull_d/include;/home/jbs/lib/cgal/Apollonius_graph_2/include;/home/jbs/lib/cgal/Polygon/include;/home/jbs/lib/cgal/BGL/include;/home/jbs/lib/cgal/Surface_mesher/include;/home/jbs/lib/cgal/Linear_cell_complex/include;/home/jbs/lib/cgal/Number_types/include;/home/jbs/lib/cgal/Advancing_front_surface_reconstruction/include;/home/jbs/lib/cgal/Combinatorial_map/include;/home/jbs/lib/cgal/Point_set_processing_3/include;/home/jbs/lib/cgal/Profiling_tools/include;/home/jbs/lib/cgal/Testsuite/include;/home/jbs/lib/cgal/Point_set_2/include;/home/jbs/lib/cgal/Random_numbers/include;/home/jbs/lib/cgal/LEDA/include;/home/jbs/lib/cgal/Optimisation_basic/include;/home/jbs/lib/cgal/TDS_3/include;/home/jbs/lib/cgal/Arithmetic_kernel/include;/home/jbs/lib/cgal/NewKernel_d/include;/home/jbs/lib/cgal/Circulator/include;/home/jbs/lib/cgal/Surface_mesh_shortest_path/include;/home/jbs/lib/cgal/Algebraic_kernel_for_spheres/include;/home/jbs/lib/cgal/Polyline_simplification_2/include;/home/jbs/lib/cgal/Distance_3/include;/home/jbs/lib/cgal/Nef_S2/include;/home/jbs/lib/cgal/Arrangement_on_surface_2/include;/home/jbs/lib/cgal/Spatial_searching/include;/home/jbs/lib/cgal/Surface_mesh_simplification/include;/home/jbs/lib/cgal/TDS_2/include;/home/jbs/lib/cgal/Set_movable_separability_2/include;/home/jbs/lib/cgal/Interpolation/include;/home/jbs/lib/cgal/Boolean_set_operations_2/include;/home/jbs/lib/cgal/Bounding_volumes/include;/home/jbs/lib/cgal/Partition_2/include;/home/jbs/lib/cgal/Ridges_3/include;/home/jbs/lib/cgal/Circular_kernel_3/include;/home/jbs/lib/cgal/Snap_rounding_2/include;/home/jbs/lib/cgal/Point_set_3/include;/home/jbs/lib/cgal/Inscribed_areas/include;/home/jbs/lib/cgal/Point_set_shape_detection_3/include;/home/jbs/lib/cgal/Triangulation/include;/home/jbs/lib/cgal/Polynomial/include;/home/jbs/lib/cgal/Interval_support/include;/home/jbs/lib/cgal/Intersections_2/include;/home/jbs/lib/cgal/Interval_skip_list/include;/home/jbs/lib/cgal/Triangulation_2/include;/home/jbs/lib/cgal/Barycentric_coordinates_2/include;/home/jbs/lib/cgal/Alpha_shapes_2/include;/home/jbs/lib/cgal/Geomview/include;/home/jbs/lib/cgal/Surface_mesh/include;/home/jbs/lib/cgal/CGAL_Core/include;/home/jbs/lib/cgal/HalfedgeDS/include;/home/jbs/lib/cgal/Surface_mesh_segmentation/include;/home/jbs/lib/cgal/Periodic_3_triangulation_3/include;/home/jbs/lib/cgal/Envelope_2/include;/home/jbs/lib/cgal/Stream_support/include;/home/jbs/lib/cgal/Skin_surface_3/include;/home/jbs/lib/cgal/Alpha_shapes_3/include " STREQUAL " ")
  set(traj_gen_INCLUDE_DIRS "")
  set(_include_dirs "include;/home/jbs/lib/cgal/SearchStructures/include;/home/jbs/lib/cgal/Surface_sweep_2/include;/home/jbs/lib/cgal/Solver_interface/include;/home/jbs/lib/cgal/Algebraic_kernel_for_circles/include;/home/jbs/lib/cgal/Matrix_search/include;/home/jbs/lib/cgal/Algebraic_foundations/include;/home/jbs/lib/cgal/Triangulation_3/include;/home/jbs/lib/cgal/OpenNL/include;/home/jbs/lib/cgal/Voronoi_diagram_2/include;/home/jbs/lib/cgal/Kernel_23/include;/home/jbs/lib/cgal/Segment_Delaunay_graph_Linf_2/include;/home/jbs/lib/cgal/Periodic_3_mesh_3/include;/home/jbs/lib/cgal/Mesh_2/include;/home/jbs/lib/cgal/Heat_method_3/include;/home/jbs/lib/cgal/Convex_hull_3/include;/home/jbs/lib/cgal/Distance_2/include;/home/jbs/lib/cgal/Polygon_mesh_processing/include;/home/jbs/lib/cgal/Mesher_level/include;/home/jbs/lib/cgal/Polytope_distance_d/include;/home/jbs/lib/cgal/Segment_Delaunay_graph_2/include;/home/jbs/lib/cgal/Cartesian_kernel/include;/home/jbs/lib/cgal/Modular_arithmetic/include;/home/jbs/lib/cgal/Nef_3/include;/home/jbs/lib/cgal/Surface_mesh_deformation/include;/home/jbs/lib/cgal/Scale_space_reconstruction_3/include;/home/jbs/lib/cgal/Inventor/include;/home/jbs/lib/cgal/Stream_lines_2/include;/home/jbs/lib/cgal/Kernel_d/include;/home/jbs/lib/cgal/AABB_tree/include;/home/jbs/lib/cgal/Convex_hull_2/include;/home/jbs/lib/cgal/Poisson_surface_reconstruction_3/include;/home/jbs/lib/cgal/Polyhedron/include;/home/jbs/lib/cgal/Envelope_3/include;/home/jbs/lib/cgal/CGAL_ipelets/include;/home/jbs/lib/cgal/Union_find/include;/home/jbs/lib/cgal/Property_map/include;/home/jbs/lib/cgal/Surface_mesh_skeletonization/include;/home/jbs/lib/cgal/Filtered_kernel/include;/home/jbs/lib/cgal/Minkowski_sum_3/include;/home/jbs/lib/cgal/Polyhedron_IO/include;/home/jbs/lib/cgal/Installation/include;/home/jbs/lib/cgal/Cone_spanners_2/include;/home/jbs/lib/cgal/QP_solver/include;/home/jbs/lib/cgal/Straight_skeleton_2/include;/home/jbs/lib/cgal/Intersections_3/include;/home/jbs/lib/cgal/Generator/include;/home/jbs/lib/cgal/Principal_component_analysis/include;/home/jbs/lib/cgal/Generalized_map/include;/home/jbs/lib/cgal/GraphicsView/include;/home/jbs/lib/cgal/Circular_kernel_2/include;/home/jbs/lib/cgal/Optimal_transportation_reconstruction_2/include;/home/jbs/lib/cgal/CGAL_ImageIO/include;/home/jbs/lib/cgal/Homogeneous_kernel/include;/home/jbs/lib/cgal/Hash_map/include;/home/jbs/lib/cgal/Subdivision_method_3/include;/home/jbs/lib/cgal/Spatial_sorting/include;/home/jbs/lib/cgal/Modifier/include;/home/jbs/lib/cgal/Classification/include;/home/jbs/lib/cgal/Surface_mesh_parameterization/include;/home/jbs/lib/cgal/Algebraic_kernel_d/include;/home/jbs/lib/cgal/Three/include;/home/jbs/lib/cgal/Periodic_2_triangulation_2/include;/home/jbs/lib/cgal/Visibility_2/include;/home/jbs/lib/cgal/Minkowski_sum_2/include;/home/jbs/lib/cgal/Principal_component_analysis_LGPL/include;/home/jbs/lib/cgal/Convex_decomposition_3/include;/home/jbs/lib/cgal/STL_Extension/include;/home/jbs/lib/cgal/Nef_2/include;/home/jbs/lib/cgal/Box_intersection_d/include;/home/jbs/lib/cgal/Mesh_3/include;/home/jbs/lib/cgal/Jet_fitting_3/include;/home/jbs/lib/cgal/Convex_hull_d/include;/home/jbs/lib/cgal/Apollonius_graph_2/include;/home/jbs/lib/cgal/Polygon/include;/home/jbs/lib/cgal/BGL/include;/home/jbs/lib/cgal/Surface_mesher/include;/home/jbs/lib/cgal/Linear_cell_complex/include;/home/jbs/lib/cgal/Number_types/include;/home/jbs/lib/cgal/Advancing_front_surface_reconstruction/include;/home/jbs/lib/cgal/Combinatorial_map/include;/home/jbs/lib/cgal/Point_set_processing_3/include;/home/jbs/lib/cgal/Profiling_tools/include;/home/jbs/lib/cgal/Testsuite/include;/home/jbs/lib/cgal/Point_set_2/include;/home/jbs/lib/cgal/Random_numbers/include;/home/jbs/lib/cgal/LEDA/include;/home/jbs/lib/cgal/Optimisation_basic/include;/home/jbs/lib/cgal/TDS_3/include;/home/jbs/lib/cgal/Arithmetic_kernel/include;/home/jbs/lib/cgal/NewKernel_d/include;/home/jbs/lib/cgal/Circulator/include;/home/jbs/lib/cgal/Surface_mesh_shortest_path/include;/home/jbs/lib/cgal/Algebraic_kernel_for_spheres/include;/home/jbs/lib/cgal/Polyline_simplification_2/include;/home/jbs/lib/cgal/Distance_3/include;/home/jbs/lib/cgal/Nef_S2/include;/home/jbs/lib/cgal/Arrangement_on_surface_2/include;/home/jbs/lib/cgal/Spatial_searching/include;/home/jbs/lib/cgal/Surface_mesh_simplification/include;/home/jbs/lib/cgal/TDS_2/include;/home/jbs/lib/cgal/Set_movable_separability_2/include;/home/jbs/lib/cgal/Interpolation/include;/home/jbs/lib/cgal/Boolean_set_operations_2/include;/home/jbs/lib/cgal/Bounding_volumes/include;/home/jbs/lib/cgal/Partition_2/include;/home/jbs/lib/cgal/Ridges_3/include;/home/jbs/lib/cgal/Circular_kernel_3/include;/home/jbs/lib/cgal/Snap_rounding_2/include;/home/jbs/lib/cgal/Point_set_3/include;/home/jbs/lib/cgal/Inscribed_areas/include;/home/jbs/lib/cgal/Point_set_shape_detection_3/include;/home/jbs/lib/cgal/Triangulation/include;/home/jbs/lib/cgal/Polynomial/include;/home/jbs/lib/cgal/Interval_support/include;/home/jbs/lib/cgal/Intersections_2/include;/home/jbs/lib/cgal/Interval_skip_list/include;/home/jbs/lib/cgal/Triangulation_2/include;/home/jbs/lib/cgal/Barycentric_coordinates_2/include;/home/jbs/lib/cgal/Alpha_shapes_2/include;/home/jbs/lib/cgal/Geomview/include;/home/jbs/lib/cgal/Surface_mesh/include;/home/jbs/lib/cgal/CGAL_Core/include;/home/jbs/lib/cgal/HalfedgeDS/include;/home/jbs/lib/cgal/Surface_mesh_segmentation/include;/home/jbs/lib/cgal/Periodic_3_triangulation_3/include;/home/jbs/lib/cgal/Envelope_2/include;/home/jbs/lib/cgal/Stream_support/include;/home/jbs/lib/cgal/Skin_surface_3/include;/home/jbs/lib/cgal/Alpha_shapes_3/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'jbs <jbs@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${traj_gen_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'traj_gen' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'traj_gen' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/usr/local/${idir}'.  ${_report}")
    endif()
    _list_append_unique(traj_gen_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "traj_gen;/usr/lib/x86_64-linux-gnu/libgmp.so;/usr/lib/x86_64-linux-gnu/libmpfr.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND traj_gen_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND traj_gen_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND traj_gen_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /usr/local/lib;/home/jbs/catkin_ws/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(traj_gen_LIBRARY_DIRS ${lib_path})
      list(APPEND traj_gen_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'traj_gen'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND traj_gen_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(traj_gen_EXPORTED_TARGETS "traj_gen_generate_messages_cpp;traj_gen_generate_messages_eus;traj_gen_generate_messages_lisp;traj_gen_generate_messages_nodejs;traj_gen_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${traj_gen_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "message_generation;nav_msgs;roscpp")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 traj_gen_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${traj_gen_dep}_FOUND)
      find_package(${traj_gen_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${traj_gen_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(traj_gen_INCLUDE_DIRS ${${traj_gen_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(traj_gen_LIBRARIES ${traj_gen_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${traj_gen_dep}_LIBRARIES})
  _list_append_deduplicate(traj_gen_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(traj_gen_LIBRARIES ${traj_gen_LIBRARIES})

  _list_append_unique(traj_gen_LIBRARY_DIRS ${${traj_gen_dep}_LIBRARY_DIRS})
  list(APPEND traj_gen_EXPORTED_TARGETS ${${traj_gen_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "traj_gen-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${traj_gen_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
