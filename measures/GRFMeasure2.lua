function init( model, par )
    -- This function is called at the start of the simulation
    -- 'model' can be used to initialize the measure parameters (see LuaModel)
     
    mass = model:mass()
    gravity = model:gravity()
    gravity_y = gravity.y

    calcn_l = model:find_body("calcn_l")
    calcn_r = model:find_body("calcn_r")

    -- grf_threshold = 1.28
    peak1_upper_limit = 1.23 -- 1.18
    peak2_upper_limit = 1.23 -- 1.23
    peak1_lower_limit = 0.95 -- 0.95
    peak2_lower_limit = 1.05 -- 1.1

    cost = 0.0
    cost1 = 0.0
    cost2 = 0.0
    cost3 = 0.0

    prev_grf_l = 0.0
    prev_grf_r = 0.0
    derivative_l = 0.0
    derivative_r = 0.0
    prev_derivative_l = 0.0
    prev_derivative_r = 0.0

end

function update( model, time )
    -- This function is called at each simulation timestep
    
    -- Calculate current GRF values
    GRF_l = calcn_l:contact_force()
    GRF_l_y = GRF_l.y
    normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    
    GRF_r = calcn_r:contact_force()
    GRF_r_y = GRF_r.y
    normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )

    -- Calculate derivatives (change rates)
    -- if not prev_grf_l then
    --     -- Initialize previous values on first run
    --     prev_grf_l = normalized_GRF_l_y
    --     prev_grf_r = normalized_GRF_r_y
    --     prev_derivative_l = 0.0
    --     prev_derivative_r = 0.0
    --     return false
    -- end

    -- Calculate current derivatives
    derivative_l = normalized_GRF_l_y - prev_grf_l
    derivative_r = normalized_GRF_r_y - prev_grf_r

    -- Detect peaks using derivative sign change
    -- Peak occurs when derivative changes from positive to negative
    if prev_derivative_l > 0 and derivative_l < 0 then
        -- Left foot peak detected
        if normalized_GRF_r_y < 0.1 then  
            if calcn_l:com_pos().x > calcn_r:com_pos().x then -- early stance phase peak1
                if normalized_GRF_l_y < peak1_lower_limit or normalized_GRF_l_y > peak1_upper_limit then
                    cost1 = cost1 + 1.0
                end
            elseif calcn_l:com_pos().x < calcn_r:com_pos().x then -- late stance phase peak2
                if normalized_GRF_l_y < peak2_lower_limit or normalized_GRF_l_y > peak2_upper_limit then
                    cost2 = cost2 + 1.0
                end
            end
        end
    end

    if prev_derivative_r > 0 and derivative_r < 0 then
        -- Right foot peak detected
        if normalized_GRF_l_y < 0.1 then  
            if calcn_r:com_pos().x > calcn_l:com_pos().x then -- early stance phase peak1
                if normalized_GRF_r_y < peak1_lower_limit or normalized_GRF_r_y > peak1_upper_limit then
                    cost1 = cost1 + 1.0
                end
            elseif calcn_r:com_pos().x < calcn_l:com_pos().x then -- late stance phase peak2
                if normalized_GRF_r_y < peak2_lower_limit or normalized_GRF_r_y > peak2_upper_limit then
                    cost2 = cost2 + 1.0
                end
            end
        end
    end

    -- Additional cost for simultaneous decrease in both feet
    if derivative_l < 0 and derivative_r < 0 then
        cost3 = cost3 + 0.5
    end

    -- Store current values for next iteration
    prev_grf_l = normalized_GRF_l_y
    prev_grf_r = normalized_GRF_r_y
    prev_derivative_l = derivative_l
    prev_derivative_r = derivative_r
    
    return false
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