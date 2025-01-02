function init( model, par, side )
    -- Global parameters
    mass = model:mass()
    gravity = model:gravity()
    gravity_y = gravity.y
    
    -- Define muscle configurations with parameters
    muscle_configs = {
        -- rectus femoris settings - from RFController2.lua
        {
            name = "rect_fem", 
            L0_mean = 0.72, L0_std = 0.01, L0_min = 0.7, L0_max = 0.8,
            c_mean = 0.5, c_std = 0.1, c_min = 0, c_max = 1.0, 
            alpha = 20.0
        },
        
        -- gluteus medius settings - from GMController1.lua and got modified
        {
            name = "glut_med", 
            L0_mean = 0.7, L0_std = 0.01, L0_min = 0.0, L0_max = 1.0,
            c_mean = 1.0, c_std = 0.01, c_min = 0.0, c_max = 2.0, 
            alpha = 3.0
        }, -- although in GMController1 these parameters must work with VelocityReflex of gluteus medius, but here you must remove the velocity reflex
        -- {
        --     name = "glut_med", 
        --     L0_mean = 0.55, L0_std = 0.01, L0_min = 0.0, L0_max = 1.0,
        --     c_mean = 1.0, c_std = 0.01, c_min = 0.0, c_max = 2.0, 
        --     alpha = 2.0
        -- },
        -- {
        --     name = "glut_med", 
        --     L0_mean = 0.68, L0_std = 0.01, L0_min = 0.0, L0_max = 1.0,
        --     c_mean = 1.0, c_std = 0.01, c_min = 0.0, c_max = 2.0, 
        --     alpha = 3.0
        -- },
        -- {
        --     name = "glut_med", 
        --     L0_mean = 0.8, L0_std = 0.01, L0_min = 0.0, L0_max = 1.0,
        --     c_mean = 1.0, c_std = 0.1, c_min = 0.0, c_max = 5.0, 
        --     alpha = 4.0
        -- }, -- only the parameters here are from GMController2.lua but they are not working
        
        -- Hamstrings - from HamstringsController2.lua
        {
            name = "hamstrings", 
            L0_mean = 0.78, L0_std = 0.01, L0_min = 0.6, L0_max = 1.0,
            c_mean = 0.5, c_std = 0.1, c_min = 0.0, c_max = 5.0,
            alpha = 1.0
        }
    }
    
    -- Initialize storage
    muscles = {}
    params = {}
    
    -- Find required bodies
    calcn_l = model:find_body("calcn_l")
    calcn_r = model:find_body("calcn_r")
    pelvis_tilt = model:find_dof("pelvis_tilt")
    
    -- states for rectus femoris
    length_descending_left = true
    length_descending_right = true
    
    -- Initialize muscles and parameters
    for _, config in ipairs(muscle_configs) do
        local muscle_name = config.name
        
        -- Store muscle objects
        muscles[muscle_name] = {
            left = model:find_muscle(muscle_name .. "_l"),
            right = model:find_muscle(muscle_name .. "_r")
        }
        
        -- Create parameters
        params[muscle_name] = {
            c = par:create_from_mean_std(muscle_name .. ".c", 
                config.c_mean, config.c_std, config.c_min, config.c_max),
            L0 = par:create_from_mean_std(muscle_name .. ".L0", 
                config.L0_mean, config.L0_std, config.L0_min, config.L0_max),
            alpha = config.alpha
        }
    end
end

-- rectus femoris control
function control_rectus_femoris(muscle_pair, params, normalized_GRF_l_y, normalized_GRF_r_y)
    local L_l = muscle_pair.left:normalized_fiber_length()
    local L_r = muscle_pair.right:normalized_fiber_length()
    
    -- update length descending mark
    if L_l < params.L0 then
        length_descending_left = false
    end
    if L_r < params.L0 then
        length_descending_right = false
    end
    
    -- calculate FMC
    local FMC_l = length_descending_left and 
                  (params.c * normalized_GRF_l_y * (L_l - params.L0)) or 0
    local FMC_r = length_descending_right and 
                  (params.c * normalized_GRF_r_y * (L_r - params.L0)) or 0
    
    return {
        left = params.alpha * FMC_l,
        right = params.alpha * FMC_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

-- gluteus medius control
function control_gluteus_medius(muscle_pair, params, normalized_GRF_l_y, normalized_GRF_r_y)
    local L_l = muscle_pair.left:normalized_fiber_length()
    local L_r = muscle_pair.right:normalized_fiber_length()
    
    local FMC_l = (L_l - params.L0) >= 0 and 
                  (params.c * normalized_GRF_l_y * (L_l - params.L0)) or 0
    local FMC_r = (L_r - params.L0) >= 0 and 
                  (params.c * normalized_GRF_r_y * (L_r - params.L0)) or 0
    
    return {
        left = params.alpha * FMC_l,
        right = params.alpha * FMC_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

-- hamstrings control
function control_hamstrings(muscle_pair, params, normalized_GRF_l_y, normalized_GRF_r_y)
    local L_l = muscle_pair.left:normalized_fiber_length()
    local L_r = muscle_pair.right:normalized_fiber_length()
    
    local FMC_l = (L_l - params.L0) >= 0 and 
                  (params.c * normalized_GRF_l_y * (L_l - params.L0)) or 0
    local FMC_r = (L_r - params.L0) >= 0 and 
                  (params.c * normalized_GRF_r_y * (L_r - params.L0)) or 0
    
    return {
        left = params.alpha * FMC_l,
        right = params.alpha * FMC_r,
        FMC_left = FMC_l,
        FMC_right = FMC_r
    }
end

function update(model, time, controller)
    -- GRF
    local GRF_l = calcn_l:contact_force()
    local GRF_l_y = GRF_l.y
    local normalized_GRF_l_y = GRF_l_y / (-gravity_y * mass)
    
    local GRF_r = calcn_r:contact_force()
    local GRF_r_y = GRF_r.y
    local normalized_GRF_r_y = GRF_r_y / (-gravity_y * mass)
    
    -- reset states for rectus femoris controller
    if normalized_GRF_l_y <= 0 then
        length_descending_left = true
    end
    if normalized_GRF_r_y <= 0 then
        length_descending_right = true
    end
    
    -- control for all muscles
    activations = {}
    for muscle_name, muscle_pair in pairs(muscles) do
        local muscle_params = params[muscle_name]
        local result
        
        -- select controller for different muscles
        if muscle_name == "rect_fem" then
            result = control_rectus_femoris(muscle_pair, muscle_params, 
                     normalized_GRF_l_y, normalized_GRF_r_y)
        elseif muscle_name == "glut_med" then
            result = control_gluteus_medius(muscle_pair, muscle_params, 
                     normalized_GRF_l_y, normalized_GRF_r_y)
        elseif muscle_name == "hamstrings" then
            result = control_hamstrings(muscle_pair, muscle_params, 
                     normalized_GRF_l_y, normalized_GRF_r_y)
        end
        
        if result then
            
            activations[muscle_name] = {
                left = result.left,
                right = result.right
            }
            
            
            muscle_pair.left:add_input(result.left)
            muscle_pair.right:add_input(result.right)
        end
    end
    
    return false
end

function store_data(current_frame)
    -- store FMC values and print as graphics
    for muscle_name, activation_pair in pairs(activations) do
        current_frame:set_value(muscle_name .. "_l.FMC", activation_pair.left)
        current_frame:set_value(muscle_name .. "_r.FMC", activation_pair.right)
    end
end