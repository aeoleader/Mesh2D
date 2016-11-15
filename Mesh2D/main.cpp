//
//  main.cpp
//  Mesh2D
//
//  Created by jxkj on 11/11/2016.
//  Copyright © 2016 xiangyu. All rights reserved.
//

#include <iostream>
#include <fstream>
#include "CGAL_Triangulation_2D.h"

//GLM mathematics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_mesh_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT> CDTP;
int main(int argc, const char * argv[]) {
//    std::ifstream infile("Points2.txt");
//    std::string line;
//    std::string delimiter = ",";
//    std::vector<Point> data;
//    std::vector<int> identifier;
//    if (infile.is_open())
//    {
//        //size_t pos = 0;
//        std::string token;
//        while (std::getline(infile, line))
//        {
//            float x = std::stof(line.substr(0, line.find(delimiter)));
//            line.erase(0, line.find(delimiter) + delimiter.length());
//            float y = std::stof(line.substr(0, line.find(delimiter)));
//            line.erase(0, line.find(delimiter) + delimiter.length());
//            int id = std::stoi(line.substr(0, line.find(delimiter)));
//            data.push_back(Point(x, y));
//            identifier.push_back(id);
//            //std::cout << x << " " << y << " " << id << std::endl;
//        }
//        infile.close();
//    }
    
    Points data;
    Identifiers identifier;
    
    data.push_back(Point(-4,0));
    identifier.push_back(1);
    data.push_back(Point(0,-4));
    identifier.push_back(1);
    data.push_back(Point(4,0));
    identifier.push_back(1);
    data.push_back(Point(0,4));
    identifier.push_back(1);
    data.push_back(Point(-4, 0));
    identifier.push_back(1);

    //Vertex_handle ve = cdt.insert(Point(4, 0));
    //Vertex_handle vf = cdt.insert(Point(10, 10));
    
    //    Vertex_handle ve = cdt.insert(Point(-2, -2));
    //    Vertex_handle vf = cdt.insert(Point(2, -2));
    //    Vertex_handle vg = cdt.insert(Point(2, 2));
    //    Vertex_handle vh = cdt.insert(Point(-2, 2));
    
    //cdt.insert_constraint(ve, vd);
    

    
    Points constraints;
    
    for (int i = 0; i < 100; ++i)
    {
        float degree = i*(glm::two_pi<float>()/100);
        constraints.push_back(Point(glm::cos(degree), glm::sin(degree)));
    }
    
    CGAL_Triangulation_2D mesh(data, identifier, 0.5);
    std::cout << "Number of faces : " << mesh.getMesh().number_of_faces() << std::endl;
    std::cout << "Number of vertices : " << mesh.getMesh().number_of_vertices() << std::endl;
    CDTP result = mesh.addConstraints(constraints);
    std::cout << "Number of faces : " << result.number_of_faces() << std::endl;
    std::cout << "Number of vertices : " << result.number_of_vertices() << std::endl;
    
    CDTP::Finite_faces_iterator fit;
    int counter = 0;
    for (fit = result.finite_faces_begin(); fit != result.finite_faces_end(); ++fit)
        if (fit->is_in_domain())
            ++counter;
    std::cout << "Number of faces in domain : " << counter << std::endl;
    std::cout << "Number of constraints : " << result.number_of_constraints() << std::endl;
    std::cout << "Number of subsonstraints : " << result.number_of_subconstraints() << std::endl;
    
    CDTP::Constraint_iterator cit;
    for (cit = result.constraints_begin(); cit != result.constraints_end(); ++cit)
    {
        std::cout << cit->Constraint_id::Constraint_id; 
    }
    return 0;
}
