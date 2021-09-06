%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [object_list, color_list] = createOptimalShape(angle_list, translation_list, vertices)    
    %% Description: 
    % required: 
    %         opts_obs(i).rpy
    %         opts_obs(i).xyz
    
    %% Create object list
    assert(size(angle_list,1)==size(translation_list,1), "num_angle and num_translation are not the same")
    num_obj = size(angle_list,1);    
    object_list(num_obj) = struct("object_vertices", [], "object_vertices_mat", [], ...
                                  "object_vertices_mat_h", [], "target_size", [], ...
                                  "rpy", [], "xyz", [], "H", [], "H_inv", [], ...
                                  "normal", [], "centroid", [], "connected_lines", []);
    color_list = getColors(num_obj);
    
    %% Create objects
    target_size = (max(vertices(3,:)) - min(vertices(3,:)));
    for i = 1:num_obj
        object_mat_h = converToHomogeneousCoord(vertices);
        rpy = angle_list(1,:); % in degree
        xyz = translation_list(1,:);
        object_list(i) = constructObjectStructure(target_size, object_mat_h, rpy, xyz);
    end
end
