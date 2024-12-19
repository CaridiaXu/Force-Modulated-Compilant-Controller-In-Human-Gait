function init( model, par, side )
    -- This function is called at the start of the simulation
    -- 'model' can be used to initialize the desired actuators (see LuaModel)
    -- 'par' can be used to define parameters for optimization (see LuaPar)
    -- 'side' denotes if the controller is for a specific side (-1 = left, 0 = unspecified, 1 = right)
    
    -- Global parameters
    mass = model:mass()
    -- alpha = par:create_from_mean_std("alpha", 5.0, 0.5, 0.0, 10.0)
    -- alpha = 5.0 / mass
    -- alpha = 1.0
    
    gravity = model:gravity()
    gravity_y = gravity.y
    
    -- Define muscle pairs and their default L0 values
    -- muscle_configs = {
    --     {name = "rect_fem", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "add_mag", L0 = 0.11, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "glut_med", L0 = 0.14, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "hamstrings", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "iliopsoas", L0 = 0.26, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0}
    -- }
    
    muscle_configs = {
        {name = "rect_fem", L0_mean = 0.87, L0_std = 0.001, L0_min = 0.85, L0_max = 0.9, c_mean = 0.5, c_std = 0.01, c_min = 0, c_max = 1.0, alpha = 1.0 },
        -- {name = "rect_fem", L0 = 0.8, alpha = 1.0, min = 0.75, max = 1.0},
        -- {name = "add_mag", L0 = 0.11, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
        {name = "glut_med", L0_mean = 0.5, L0_std = 0.01, L0_min = 0.0, L0_max = 2.0, c_mean = 0.5, c_std = 0.01, c_min = 0, c_max = 10.0, alpha = -0.3},
        {name = "hamstrings", L0_mean = 0.82, L0_std = 0.01, L0_min = 0.0, L0_max = 2.0, c_mean = 2.0, c_std = 0.1, c_min = 0.0, c_max = 5.0,alpha = 1.0 }   -- 0.78 0.85 0.79
        -- {name = "hamstrings", L0 = 0.78, alpha = 1.0, min = 0.6, max = 1.0}   -- 0.78 0.85
        -- {name = "iliopsoas", L0 = 0.26, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0}
    }
    
    -- Initialize storage for muscle objects and parameters
    muscles = {}
    params = {}
    
    -- Find calcn bodies and pelvis tilt
    calcn_l = model:find_body("calcn_l")
    calcn_r = model:find_body("calcn_r")
    pelvis_tilt = model:find_dof("pelvis_tilt")
    
    -- Initialize each muscle pair and their parameters
    for _, config in ipairs(muscle_configs) do
        local muscle_name = config.name
        local L0_mean = config.L0_mean
        local L0_std = config.L0_std
        local L0_min =  config.L0_min
        local L0_max =  config.L0_max
        local c_mean = config.c_mean
        local c_std = config.c_std
        local c_min = config.c_min
        local c_max = config.c_max
        
        -- Store muscle objects
        muscles[muscle_name] = {
            left = model:find_muscle(muscle_name .. "_l"),
            right = model:find_muscle(muscle_name .. "_r")
        }
        
        -- Create single parameter set for each muscle
        params[muscle_name] = {
            c = par:create_from_mean_std(muscle_name .. ".c", c_mean, c_std, c_min, c_max),
            L0 = par:create_from_mean_std(muscle_name .. ".L0", L0_mean, L0_std, L0_min, L0_max),
            alpha = config.alpha 
        }
    end
    
    -- KL_r = par:create_from_mean_std( "hamstrings_r.KL", 1.0, 0.01, 0.0, 10.0)
    -- KL = par:create_from_mean_std( "hamstrings.KL", 1.0, 0.01, 0.0, 2.0)
    -- KV = par:create_from_mean_std( "hamstrings.KV", 0.21, 0.002, 0.0, 2.0)
    glut_med_L_r_min = 2.0
    glut_med_L_l_min = 2.0

end

-- Control function for rectus femoris muscle
function control_rectus_femoris(muscle_pair, params, normalized_GRF_l, normalized_GRF_r)
    -- Left side control
    local L_l = muscle_pair.left:normalized_fiber_length()
    local FMC_l = params.c * normalized_GRF_l * (L_l - params.L0)
    local activation_l = params.alpha * FMC_l
    
    -- Right side control
    local L_r = muscle_pair.right:normalized_fiber_length()
    local FMC_r = params.c * normalized_GRF_r * (L_r - params.L0)
    local activation_r = params.alpha * FMC_r
    
    return {
        left = activation_l,
        right = activation_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

-- Control function for hamstrings
function control_hamstrings(muscle_pair, params, normalized_GRF_l, normalized_GRF_r)
    -- Left side control with hamstring-specific logic
    local L_l = muscle_pair.left:normalized_fiber_length()
    local FMC_l = 0
    if (L_l - params.L0) >= 0 then
        -- Here you can modify the control logic specifically for hamstrings
        FMC_l = params.c * normalized_GRF_l * (L_l - params.L0)
    else
        FMC_l = 0
    end
    local activation_l = params.alpha * FMC_l
    
    -- Right side control with hamstring-specific logic
    local L_r = muscle_pair.right:normalized_fiber_length()
    local FMC_r = 0
    if (L_r - params.L0) >= 0 then
        -- Here you can modify the control logic specifically for hamstrings
        FMC_r = params.c * normalized_GRF_r * (L_r - params.L0)
    else
        FMC_r = 0
    end
    local activation_r = params.alpha * FMC_r
    
    return {
        left = activation_l,
        right = activation_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

function control_gluteus_medius(muscle_pair, params, normalized_GRF_l, normalized_GRF_r)
    -- Left side control
    local L_l = muscle_pair.left:normalized_fiber_length()
    local FMC_l = 0
    
    if (L_l - params.L0) >= 0 then
        -- Here you can modify the control logic specifically for hamstrings
        FMC_l = params.c * normalized_GRF_l * (L_l - params.L0)
    else
        FMC_l = 0
    end

    -- Only activate when there is ground reaction force and length is decreasing
    -- if normalized_GRF_l > 0 then
    --     -- Store the minimum length when GRF first appears
    --     if L_l < glut_med_L_l_min then
    --         glut_med_L_l_min = L_l
    --     -- Only apply control while length is decreasing
    --         FMC_l = params.c * normalized_GRF_l * ( L_l - params.L0 )
    --     else
    --         FMC_l = 0
    --     end
    -- else
    --     -- Reset initial length when no ground contact
    --     glut_med_L_l_min = 2.0
    -- end
    
    -- Right side control (similar logic)
    local L_r = muscle_pair.right:normalized_fiber_length()
    local FMC_r = 0
    
    if (L_r - params.L0) >= 0 then
        -- Here you can modify the control logic specifically for hamstrings
        FMC_r = params.c * normalized_GRF_r * (L_r - params.L0)
    else
        FMC_r = 0
    end

    -- if normalized_GRF_r > 0 then
    --     if L_r < glut_med_L_r_min then
    --         glut_med_L_r_min = L_r
    --         FMC_r = params.c * normalized_GRF_r * ( L_r - params.L0 )
    --     else
    --         FMC_r = 0
    --     end
    -- else
    --     glut_med_L_r_min = 2.0
    -- end
    
    -- Apply the activation scaling factor
    local activation_l = params.alpha * FMC_l
    local activation_r = params.alpha * FMC_r
    
    return {
        left = activation_l,
        right = activation_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

-- update function
function update(model, time, controller)
    -- Get ground reaction forces
    local GRF_l = calcn_l:contact_force()
    local GRF_l_y = GRF_l.y
    local normalized_GRF_l_y = GRF_l_y / (-gravity_y * mass)
    
    local GRF_r = calcn_r:contact_force()
    local GRF_r_y = GRF_r.y
    local normalized_GRF_r_y = GRF_r_y / (-gravity_y * mass)
    
    -- Process each muscle using its specific control function
    activations = {}
    for muscle_name, muscle_pair in pairs(muscles) do
        local muscle_params = params[muscle_name]
        local result
        
        if muscle_name == "rect_fem" then
            result = control_rectus_femoris(muscle_pair, muscle_params, normalized_GRF_l_y, normalized_GRF_r_y)
        elseif muscle_name == "hamstrings" then
            result = control_hamstrings(muscle_pair, muscle_params, normalized_GRF_l_y, normalized_GRF_r_y)
        elseif muscle_name == "glut_med" then
            result = control_gluteus_medius(muscle_pair, muscle_params, normalized_GRF_l_y, normalized_GRF_r_y)
        end
        
        if result then
            -- Store activations for data logging
            activations[muscle_name] = {
                left = result.left,
                right = result.right
            }
            
            -- Apply activations
            muscle_pair.left:add_input(result.left)
            muscle_pair.right:add_input(result.right)
        end
    end
    
    return false
end

-- function update( model, time, controller )
--     -- This function is called at each simulation timestep
--     -- Use it to update the actuator inputs
--     -- 'model' can be used to update the desired actuators (see LuaModel)
--     -- 'time' is the time elapsed since this controller was activated
--     -- 'controller' can be used to enable/disable child controllers of this ScriptController (see LuaController)
    
--     -- Get ground reaction forces
--     local GRF_l = calcn_l:contact_force()
--     local GRF_l_y = GRF_l.y
--     local normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    
--     local GRF_r = calcn_r:contact_force()
--     local GRF_r_y = GRF_r.y
--     local normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )
    
--     -- Calculate and apply activations for each muscle
--     activations = {}
--     for muscle_name, muscle_pair in pairs(muscles) do
        
--         if muscle_name = "rect_fem" then
--             -- Get muscle parameters
--             local muscle_params = params[muscle_name]
        
--             -- Calculate left side
--             -- L_l = muscle_pair.left:fiber_length() + muscle_pair.left:tendon_length()
--             L_l = muscle_pair.left:normalized_fiber_length()
        
--             if (L_l - muscle_params.L0) < 0 then
--                 FMC_l = 0
--             else
--                 FMC_l = muscle_params.c * normalized_GRF_l_y * ( L_l - muscle_params.L0 )
--             end
        
--             local activation_l = muscle_params.alpha * FMC_l
        
--             -- Calculate right side
--             L_r = muscle_pair.right:normalized_fiber_length()
--             -- local L_r = muscle_pair.right:fiber_length() + muscle_pair.right:tendon_length()
--             -- local L_r = muscle_pair.right:optimal_fiber_length() + muscle_pair.right:tendon_slack_length()

--             if (L_r - muscle_params.L0) < 0 then
--                 FMC_r = 0
--             else
--                 FMC_r = muscle_params.c * normalized_GRF_r_y * ( L_r - muscle_params.L0 )
--             end

--             local activation_r = muscle_params.alpha * FMC_r
        
--             -- Store activations for data logging
--             activations[muscle_name] = {
--                 left = activation_l,
--                 right = activation_r
--             }
        
--             -- Apply activations
--             muscle_pair.left:add_input(activation_l)
--             muscle_pair.right:add_input(activation_r)
        
--     end
    
--     -- RL_l = KL * ( muscles["hamstrings"].left:tendon_length() + muscles["hamstrings"].left:fiber_length() - params["hamstrings"].L0 )
--     -- RL_r = KL * ( muscles["hamstrings"].right:tendon_length() + muscles["hamstrings"].right:fiber_length() - params["hamstrings"].L0 )
--     -- RV_l = KV * ( muscles["hamstrings"].left:normalized_fiber_velocity() )
--     -- RV_r = KV * ( muscles["hamstrings"].right:normalized_fiber_velocity() )
    
--     -- muscles["hamstrings"].left:add_input( RL_l )
--     -- muscles["hamstrings"].right:add_input( RL_r )
--     -- muscles["hamstrings"].left:add_input( RV_l )
--     -- muscles["hamstrings"].right:add_input( RV_r )
    
--     return false -- change to 'return true' to terminate the simulation early
-- end

function store_data( current_frame )
    -- This function is called at each simulation timestep
    -- 'current_frame' can be used to store values for analysis (see LuaFrame)
    
    -- Store FMC values for all muscles
    for muscle_name, activation_pair in pairs(activations) do
        current_frame:set_value(muscle_name .. "_l.FMC", activation_pair.left)
        current_frame:set_value(muscle_name .. "_r.FMC", activation_pair.right)
    end

    -- current_frame:set_value("hamstrings_l_RL", RL_l)
    -- current_frame:set_value("hamstrings_r_RL", RL_r)
    -- current_frame:set_value("hamstrings_l_RV", RV_l)
    -- current_frame:set_value("hamstrings_r_RV", RV_r)

end