function init( model, par )
    -- This function is called at the start of the simulation
    -- 'model' can be used to initialize the measure parameters (see LuaModel)
     
    mass = model:mass()
    gravity = model:gravity()
    gravity_y = gravity.y

    calcn_l = model:find_body("calcn_l")
    calcn_r = model:find_body("calcn_r")

    -- grf_threshold = 1.28
    peak1_upper_limit = 1.28
    peak2_upper_limit = 1.28
    peak1_lower_limit = 0.9
    peak2_lower_limit = 1.0

    cost = 0.0
    cost1 = 0.0
    cost2 = 0.0
    cost3 = 0.0

    prev_grf_l = 0.0
    prev_grf_r = 0.0

end

function update( model, time )
    -- This function is called at each simulation timestep
    -- Use it to update the internal variables of the measure (if needed)
    
    GRF_l = calcn_l:contact_force()
    GRF_l_y = GRF_l.y
    normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    
    GRF_r = calcn_r:contact_force()
    GRF_r_y = GRF_r.y
    normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )

    -- if normalized_GRF_l_y > grf_threshold and normalized_GRF_r_y <= 0.1 then
    --     cost1 = cost1 + 1.0
    --     -- scone.debug(normalized_GRF_l_y)
    -- elseif  normalized_GRF_r_y > grf_threshold and normalized_GRF_l_y <= 0.1 then
    --     cost1 = cost1 + 1.0
    -- end

    -- if calcn_l:com_pos().x < calcn_r:com_pos().x and normalized_GRF_l_y <= 0.1 then
    --     if normalized_GRF_r_y < 0.95 or normalized_GRF_r_y > grf_threshold then
    --         cost1 = cost1 + 1.0
    --     end
    -- elseif calcn_r:com_pos().x < calcn_l:com_pos().x and normalized_GRF_r_y <= 0.1 then
    --     if normalized_GRF_l_y < 0.95 or normalized_GRF_l_y > grf_threshold then
    --         cost1 = cost1 + 1.0
    --     end
    -- end

    if normalized_GRF_r_y < peak1_lower_limit or normalized_GRF_r_y > peak1_upper_limit then
        if calcn_l:com_pos().x < calcn_r:com_pos().x and normalized_GRF_l_y < prev_grf_l then
            cost1 = cost1 + 1.0
        end
    elseif normalized_GRF_l_y < peak1_lower_limit or normalized_GRF_l_y > peak1_upper_limit then
        if calcn_r:com_pos().x < calcn_l:com_pos().x and normalized_GRF_r_y < prev_grf_r then  
            cost1 = cost1 + 1.0
        end
    end

    if normalized_GRF_r_y < peak2_lower_limit or normalized_GRF_r_y > peak2_upper_limit then
        if calcn_l:com_pos().x > calcn_r:com_pos().x and normalized_GRF_l_y > prev_grf_l then 
            cost2 = cost2 + 1.0
        end
    elseif normalized_GRF_l_y < peak2_lower_limit or normalized_GRF_l_y > peak2_upper_limit then
        if calcn_r:com_pos().x > calcn_l:com_pos().x and normalized_GRF_r_y > prev_grf_r then
            cost2 = cost2 + 1.0
        end
    end

    -- if calcn_l:com_pos().x > calcn_r:com_pos().x and normalized_GRF_l_y <= 0.1 then
    --     if normalized_GRF_r_y < 1.05 or normalized_GRF_r_y > grf_threshold then
    --         cost2 = cost2 + 1.0
    --     end
    -- elseif calcn_r:com_pos().x > calcn_l:com_pos().x and normalized_GRF_r_y <= 0.1 then
    --     if normalized_GRF_l_y < 1.05 or normalized_GRF_l_y > grf_threshold then
    --         cost2 = cost2 + 1.0
    --     end
    -- end

    if normalized_GRF_r_y < prev_grf_r and normalized_GRF_l_y < prev_grf_l then
        cost3 = cost3 + 1.0
    end

    prev_grf_l = normalized_GRF_l_y
    prev_grf_r = normalized_GRF_r_y
    
    return false -- change to 'return true' to terminate the simulation early
end

function result( model )
    -- This function is called at the end of the simulation
    -- It should return the result of the measure

    scone.debug("cost1 = ")
    scone.debug(cost1)
    scone.debug("cost2 = ")
    scone.debug(cost2)
    scone.debug("cost3 = ")
    scone.debug(cost3)
    
    cost = cost1 + cost2 + cost3

    return cost / model:max_duration()
end

function store_data( current_frame )
    -- This function is called at each simulation timestep
    -- 'current_frame' can be used to store values for analysis (see LuaFrame)


end