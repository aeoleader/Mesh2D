//
//  CGAL_Triangulation_2D.cpp
//  CGAL_Triangulation_2D
//
//  Created by jxkj on 11/11/2016.
//  Copyright © 2016 xiangyu. All rights reserved.
//

#include "CGAL_Triangulation_2D.h"

CGAL_Triangulation_2D::CGAL_Triangulation_2D(Points pts, Identifiers ids, float density, int iteration)
{
    this->m_pts = pts;
    this->m_ids = ids;
    if (density > 1 || density < 0.05)
    {
        std::cerr << "Warning: CGAL_Triangulation_2D::m_density is out of bound and triangulation might take too long or undefined.\n";
        //std::terminate();
    }
    this->m_density = density;
    this->m_iteration = iteration;
    assert(!pts.empty() && !ids.empty());

    mesh(m_density);
}


CDTP CGAL_Triangulation_2D::lloyd_optimization(int iteration)
{
    
    if (iteration > 100 || iteration < 0)
        std::cerr << "Asseration failed: CGAL_Triangulation_2D::lloyd_optimization::iteration out of bounds.\n";
    m_iteration = iteration;
    CGAL::lloyd_optimize_mesh_2(m_mesh, CGAL::parameters::max_iteration_number = m_iteration);
    return m_mesh;
}

CDTP CGAL_Triangulation_2D::reMesh(float density, int iteration)
{
    assert(density > 0.05 && density < 1 && iteration > -1 && iteration < 100);
    m_iteration = iteration;
    m_density = density;
    m_mesh.clear();
    mesh(m_density);
    return m_mesh;
}

CDTP CGAL_Triangulation_2D::getMesh()
{
    return m_mesh;
}

void CGAL_Triangulation_2D::mesh(float density)
{
    Vertex_handle prev = m_mesh.insert(m_pts[0]);
    Vertex_handle start = prev;
    Vertex_handle current;
    int i;
    for (i = 1; i < m_pts.size(); i++)
    {
        current = m_mesh.insert(m_pts[i]);
        if (m_ids[i] == m_ids[i-1])
            m_mesh.insert_constraint(prev, current);
        else
            std::cout << glm::sqrt(glm::pow(m_pts[i-2].x() - m_pts[i-1].x(), 2) + glm::pow(m_pts[i-2].y() - m_pts[i-1].y(), 2)) << std::endl;
        prev = current;
    }
    std::cout << glm::sqrt(glm::pow(m_pts[i-2].x() - m_pts[i-1].x(), 2) + glm::pow(m_pts[i-2].y() - m_pts[i-1].y(), 2)) << std::endl;
    
    CGAL::refine_Delaunay_mesh_2(m_mesh, Criteria(0.125 , density));
    std::cout << m_mesh.number_of_vertices() << " " << m_mesh.number_of_faces() << std::endl;
    //lloyd_optimization(m_iteration);
}

CDTP CGAL_Triangulation_2D::addConstraints(Points pts)
{
    m_constraint = pts;
    CDTP constrained;
    if (!CGAL::is_simple_2(pts.begin(), pts.end(), K()))
    {
        std::cout << "The constraints is not simple " << std::endl;
        std::terminate();
    }
    
    CDTP::Finite_faces_iterator fit;
    for (fit = m_mesh.finite_faces_begin(); fit != m_mesh.finite_faces_end(); ++fit)
    {
        Point v1 = m_mesh.triangle(fit).vertex(0);
        Point v2 = m_mesh.triangle(fit).vertex(1);
        Point v3 = m_mesh.triangle(fit).vertex(2);
        bool flag = false;
        flag = check_inside(v1, K()) || check_inside(v2, K()) || check_inside(v3, K());
        if (flag) continue;
        constrained.insert(v1);
        constrained.insert(v2);
        constrained.insert(v3);
    }
    
    Vertex_handle prev = constrained.insert(pts[0]);
    Vertex_handle start = prev;
    Vertex_handle curr;
    for (int i = 1; i < pts.size(); ++i)
    {
        curr = constrained.insert(pts[i]);
        constrained.insert_constraint(prev, curr);
        prev = curr;
    }
    constrained.insert_constraint(prev, start);
    
    Points list_of_seeds;
    list_of_seeds.push_back(Point(0, 0));
    Mesher mesher(constrained);
    mesher.set_criteria(Criteria());

    mesher.set_seeds(list_of_seeds.begin(), list_of_seeds.end(), true, true);
    mesher.refine_mesh();
    return constrained;
}

bool CGAL_Triangulation_2D::check_inside(Point pt, K traits)
{
    switch(CGAL::bounded_side_2(m_constraint.begin(), m_constraint.end(),pt, traits)) {
        case CGAL::ON_BOUNDED_SIDE:
            return true;
            break;
        case CGAL::ON_BOUNDARY:
            //std::cout << " is ont the boundary of the polygon.\n";
            break;
        case CGAL::ON_UNBOUNDED_SIDE:
            //std::cout << " is outside the polygon.\n";
            break;
    }
    return false;
}
