//
//  mesh_generation.h
//
//  Created by Alp Cihan on 12.12.2020.
//  Copyright © 2020 Alp Cihan. All rights reserved.
//

#pragma once

#include <iostream>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace gfx
{

/* Generator Functions */
void GenerateParametricShapeFrom2D(
    std::vector<glm::vec3>& positions,
    std::vector<glm::vec3>& normals,
    std::vector<glm::vec2>& uvs,
    std::vector<unsigned int>& indices,
    glm::dvec2(*parametric_line)(double),
    int vertical_segments,
    int rotation_segments
);

/* Example 2D Parametric Functions */
glm::dvec2 ParametricHalfCircle(double);

}