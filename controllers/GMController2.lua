function init( model, par, side )
    -- Global parameters
    mass = model:mass()
    gravity = model:gravity()
    gravity_y = gravity.y
    
    -- Define muscle configurations
    -- muscle_configs = {
    --     {name = "glut_med", L0 = 0.8, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0}
    -- } -- 457 gen 1.459 with velocity reflex
    muscle_configs = {
        {name = "glut_med", L0 = 0.8, alpha = 4.0, std = 0.01, min = 0.0, max = 1.0}
    } -- 559 gen 1.554 without velocity reflex
    

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
        local default_L0 = config.L0
        local min = config.min
        local max = config.max
        local std = config.std
        
        -- Store muscle objects
        muscles[muscle_name] = {
            left = model:find_muscle(muscle_name .. "_l"),
            right = model:find_muscle(muscle_name .. "_r")
        }
        
        -- Create single parameter set for each muscle
        params[muscle_name] = {
            -- c = par:create_from_mean_std(muscle_name .. ".c", 1.0, 0.1, 0.0, 5.0),
            c = par:create_from_mean_std(muscle_name .. ".c", 1.0, 0.1, 0.0, 5.0),
            L0 = par:create_from_mean_std(muscle_name .. ".L0", default_L0, std, min, max),
            alpha = config.alpha 
        }
    end
    
    -- Initialize length descending flags
    length_descending_left = true
    length_descending_right = true
end

function update( model, time, controller )
    -- Get ground reaction forces
    local GRF_l = calcn_l:contact_force()
    local GRF_l_y = GRF_l.y
    local normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    
    local GRF_r = calcn_r:contact_force()
    local GRF_r_y = GRF_r.y
    local normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )
    
    -- Reset length descending flags when no ground contact
    if normalized_GRF_l_y > 0 then
    else
        length_descending_left = true
    end

    if normalized_GRF_r_y > 0 then
    else
        length_descending_right = true
    end

    -- Calculate and apply activations for each muscle
    activations = {}
    for muscle_name, muscle_pair in pairs(muscles) do
        -- Get muscle parameters
        local muscle_params = params[muscle_name]
        
        -- Calculate left side activation
        L_l = muscle_pair.left:normalized_fiber_length()
        
        if L_l < muscle_params.L0 then
            length_descending_left = false
        end

        if length_descending_left then
            FMC_l = muscle_params.c * normalized_GRF_l_y * ( L_l - muscle_params.L0 )
        else
            FMC_l = 0
        end

        local activation_l = muscle_params.alpha * FMC_l
        
        -- Calculate right side activation
        L_r = muscle_pair.right:normalized_fiber_length()

        if L_r < muscle_params.L0 then
            length_descending_right = false
        end

        if length_descending_right then
            FMC_r = muscle_params.c * normalized_GRF_r_y * ( L_r - muscle_params.L0 )
        else
            FMC_r = 0
        end

        local activation_r = muscle_params.alpha * FMC_r
        
        -- Store activations for data logging
        activations[muscle_name] = {
            left = activation_l,
            right = activation_r
        }
        
        -- Apply activations
        muscle_pair.left:add_input(activation_l)
        muscle_pair.right:add_input(activation_r)
    end
    
    return false
end

function store_data( current_frame )
    -- Store FMC values for all muscles
    for muscle_name, activation_pair in pairs(activations) do
        current_frame:set_value(muscle_name .. "_l.FMC", activation_pair.left)
        current_frame:set_value(muscle_name .. "_r.FMC", activation_pair.right)
    end
end