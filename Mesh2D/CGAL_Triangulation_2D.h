//
//  CGAL_Triangulation_2D.h
//  CGAL_Triangulation_2D
//
//  Created by jxkj on 11/11/2016.
//  Copyright © 2016 xiangyu. All rights reserved.
//
#pragma once
#ifndef CGAL_Triangulation_2D_h
#define CGAL_Triangulation_2D_h

#define CGAL_MESH_2_OPTIMIZER_VERBOSE

//std includes
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <assert.h>

//GLM mathematics
#include <glm/glm.hpp>

//CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include <CGAL/Polygon_2.h>

#include <CGAL/lloyd_optimize_mesh_2.h>

#include <CGAL/Timer.h>

//CGAL Typedefs
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_mesh_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT> CDTP;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;
typedef K::Point_2 Point;
typedef CDTP::Vertex_handle Vertex_handle;

typedef std::vector<Point> Points;
typedef std::vector<int> Identifiers;

class CGAL_Triangulation_2D
{
public:
    CGAL_Triangulation_2D(Points pts, Identifiers ids, float density, int iteration = 5);
    CDTP lloyd_optimization(int iteration);
    CDTP reMesh(float density, int iteration);
    CDTP getMesh();
    CDTP addConstraints(Points pts);
    bool check_inside(Point pt, K traits);
private:
    CDTP m_mesh;
    std::vector<CDTP> m_constrained;
    Points m_constraint;
    Points m_pts;
    Identifiers m_ids;
    float m_density;
    int m_iteration;
    
    void mesh(float density);
};

#endif /* CGAL_Triangulation_2D_h */
