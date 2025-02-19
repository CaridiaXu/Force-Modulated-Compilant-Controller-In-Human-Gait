function init( model, par, side )
    -- Global parameters
    mass = model:mass()
    gravity = model:gravity()
    gravity_y = gravity.y
    
    -- Define muscle pairs and their default L0 values
    muscle_configs = {
        {name = "hamstrings", L0 = 0.8, alpha = 1.0, min = 0.6, max = 1.0}, 
        {name = "glut_med", L0 = 0.77, alpha = 1.0, min = 0.0, max = 1.0},
        -- {name = "soleus", L0 = 0.7, alpha = 0.1, min = 0.0, max = 0.84},
        -- {name = "tib_ant", L0 = 0.7, alpha = 0.1, min = 0.0, max = 0.84},
        {name = "glut_max", L0 = 0.72, alpha = 0.4, min = 0.0, max = 0.8},
        -- {name = "iliopsoas", L0 = 0.72, alpha = 0.01, min = 0.0, max = 0.8},
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
        local default_L0 = config.L0
        local min = config.min
        local max = config.max
        
        -- Store muscle objects
        muscles[muscle_name] = {
            left = model:find_muscle(muscle_name .. "_l"),
            right = model:find_muscle(muscle_name .. "_r")
        }
        
        -- Create single parameter set for each muscle
        local c_mean = 
            muscle_name == "hamstrings" and 1.28 or
            muscle_name == "glut_med" and 2.7 or
            muscle_name == "glut_max" and 2.5 or
            muscle_name == "iliopsoas" and 1.0 or 
            muscle_name == "soleus" and 2.5

        params[muscle_name] = {
            c = par:create_from_mean_std(muscle_name .. ".c", c_mean, 0.01, 0.0, 5.0),
            L0 = par:create_from_mean_std(muscle_name .. ".L0", default_L0, 0.01, min, max),    -- 586 gen 1.754
            alpha = config.alpha 
        }
    end
    
    -- Initialize rect_fem muscle objects without parameters
    muscles["rect_fem"] = {
        left = model:find_muscle("rect_fem_l"),
        right = model:find_muscle("rect_fem_r")
    }
    gastroc_l = model:find_muscle("gastroc_l")
    gastroc_r = model:find_muscle("gastroc_r")
    
    prev_grf_l = 0.0
    prev_grf_r = 0.0
end

function update( model, time, controller )
    -- Get ground reaction forces
    local GRF_l = calcn_l:contact_force()
    local GRF_l_x = GRF_l.x  
    local GRF_l_y = GRF_l.y  
    local GRF_l_z = GRF_l.z  
    local normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    local normalized_GRF_l_x = GRF_l_x / ( -gravity_y * mass )
    local normalized_GRF_l_z = GRF_l_z / ( -gravity_y * mass )
    
    local GRF_r = calcn_r:contact_force()
    local GRF_r_x = GRF_r.x
    local GRF_r_y = GRF_r.y
    local GRF_r_z = GRF_r.z
    local normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )
    local normalized_GRF_r_x = GRF_r_x / ( -gravity_y * mass )
    local normalized_GRF_r_z = GRF_r_z / ( -gravity_y * mass )
    
    -- Calculate and apply activations for each muscle
    activations = {}
    
    -- Process each muscle
    for muscle_name, muscle_pair in pairs(muscles) do
        -- Skip rect_fem parameter calculation as it uses hamstrings
        if muscle_name == "rect_fem" then
            local hamstrings_params = params["hamstrings"]
            local hamstrings_l = muscles["hamstrings"].left:normalized_fiber_length()
            local hamstrings_r = muscles["hamstrings"].right:normalized_fiber_length()
            
            -- Calculate rect_fem activation based on hamstrings
            local FMC_l = 0
            local FMC_r = 0
            
            if (hamstrings_l - hamstrings_params.L0) >= 0 then
                FMC_l = hamstrings_params.c * normalized_GRF_l_y * (hamstrings_l - hamstrings_params.L0)
            end
            
            if (hamstrings_r - hamstrings_params.L0) >= 0 then
                FMC_r = hamstrings_params.c * normalized_GRF_r_y * (hamstrings_r - hamstrings_params.L0)
            end
            
            -- Calculate final activations
            local activation_l = hamstrings_params.alpha * FMC_l * 0.8
            local activation_r = hamstrings_params.alpha * FMC_r * 0.8
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)
        
        elseif muscle_name == "glut_max" then
            local muscle_params = params[muscle_name]
            -- 参数定义（需在params中初始化）
            local w_x = 1.0  -- vertical weight
            local w_z = 0.0  -- lateral weight
            local FMC_l = 0
            local FMC_r = 0
            
            -- 左侧激活
            local L_l = muscle_pair.left:normalized_fiber_length()
            if (L_l - muscle_params.L0) >= 0 then
            -- if normalized_GRF_l_y > 0  then
                FMC_l = muscle_params.c * ( 0 + w_x * normalized_GRF_l_y + w_z * normalized_GRF_r_z) * (L_l - muscle_params.L0)
                -- FMC_l = muscle_params.c * normalized_GRF_l_y * (L_l - muscle_params.L0)
            end
            
            if calcn_r:com_pos().x > calcn_l:com_pos().x then
                FMC_l = 0
            end
            
            -- 右侧激活（同理）
            local L_r = muscle_pair.right:normalized_fiber_length()
            if (L_r - muscle_params.L0) >= 0 then
            -- if normalized_GRF_l_y > 0  then
                FMC_r = muscle_params.c * ( 0 + w_x * normalized_GRF_r_y + w_z * normalized_GRF_l_z) * (L_r - muscle_params.L0)
                -- FMC_r = muscle_params.c * normalized_GRF_r_y * (L_r - muscle_params.L0)
            end
            if calcn_l:com_pos().x > calcn_r:com_pos().x then
                FMC_r = 0
            end

            -- Calculate final activations
            local activation_l = muscle_params.alpha * FMC_l
            local activation_r = muscle_params.alpha * FMC_r
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)
        
        elseif muscle_name == "iliopsoas" then
            local muscle_params = params[muscle_name]

            -- Calculate left side activation
            local L_l = muscle_pair.left:normalized_fiber_length()
            local FMC_l = 0
            
            if (L_l - muscle_params.L0) < 0 then
                FMC_l = 0
            else
                FMC_l = muscle_params.c * normalized_GRF_r_y * ( L_l - muscle_params.L0 )
            end
    
            -- if normalized_GRF_r_y - prev_grf_r > 0 or calcn_r:com_pos().x > 0.05 then
            if calcn_l:com_pos().x > calcn_r:com_pos().x then
                FMC_l = 0
            end
            
            -- Calculate right side activation
            local L_r = muscle_pair.right:normalized_fiber_length()
            local FMC_r = 0
            
            if (L_r - muscle_params.L0) < 0 then
                FMC_r = 0
            else
                FMC_r = muscle_params.c * normalized_GRF_l_y * ( L_r - muscle_params.L0 )
            end

            -- if normalized_GRF_l_y - prev_grf_l > 0 or calcn_l:com_pos().y < 0.05 then
            if calcn_r:com_pos().x > calcn_l:com_pos().x then
                FMC_r = 0
            end
            
            -- Calculate final activations
            local activation_l = muscle_params.alpha * FMC_l
            local activation_r = muscle_params.alpha * FMC_r
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)
            -- gastroc_l:add_input(activation_l)
            -- gastroc_r:add_input(activation_r)

        elseif muscle_name == "soleus" then
            local muscle_params = params[muscle_name]

            -- Calculate left side activation
            local L_l = muscle_pair.left:normalized_fiber_length()
            local FMC_l = 0
            
            if normalized_GRF_l_x > 0 then
                FMC_l = muscle_params.c * normalized_GRF_l_x * ( L_l - muscle_params.L0 )
            end
    
            -- if normalized_GRF_r_y - prev_grf_r > 0 or calcn_r:com_pos().x > 0.05 then
            -- if calcn_l:com_pos().x > calcn_r:com_pos().x then
            --     FMC_l = 0
            -- end
            
            -- Calculate right side activation
            local L_r = muscle_pair.right:normalized_fiber_length()
            local FMC_r = 0
            
            if normalized_GRF_r_x > 0 then
                FMC_r = muscle_params.c * normalized_GRF_r_x * ( L_r - muscle_params.L0 )
            end

            -- if normalized_GRF_l_y - prev_grf_l > 0 or calcn_l:com_pos().y < 0.05 then
            -- if calcn_r:com_pos().x > calcn_l:com_pos().x then
            --     FMC_r = 0
            -- end
            
            -- Calculate final activations
            local activation_l = muscle_params.alpha * FMC_l
            local activation_r = muscle_params.alpha * FMC_r
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)
            gastroc_l:add_input(activation_l)
            gastroc_r:add_input(activation_r)

        elseif muscle_name == "glut_med" then
            -- Get muscle parameters
            local muscle_params = params[muscle_name]
            
            -- Calculate left side activation
            local L_l = muscle_pair.left:normalized_fiber_length()
            local FMC_l = 0

            if (L_l - muscle_params.L0) >= 0 then
                FMC_l = muscle_params.c * normalized_GRF_l_y * (L_l - muscle_params.L0)
                -- FMC_l = muscle_params.c * ( normalized_GRF_l_y + normalized_GRF_l_z ) * (L_l - muscle_params.L0)
            end
            -- Special handling for glut_med
            if normalized_GRF_r_y > 0 and calcn_r:com_pos().x > calcn_l:com_pos().x then
                FMC_l = 0
            end

            -- Calculate right side activation
            local L_r = muscle_pair.right:normalized_fiber_length()
            local FMC_r = 0
            
            if (L_r - muscle_params.L0) >= 0 then
                FMC_r = muscle_params.c * normalized_GRF_r_y * (L_r - muscle_params.L0)
                -- FMC_r = muscle_params.c * ( normalized_GRF_l_y + normalized_GRF_l_z ) * (L_r - muscle_params.L0)
            end
            -- Special handling for glut_med
            if normalized_GRF_l_y > 0 and calcn_l:com_pos().x > calcn_r:com_pos().x then
                FMC_r = 0
            end
            
            -- Calculate final activations
            local activation_l = muscle_params.alpha * FMC_l
            local activation_r = muscle_params.alpha * FMC_r
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)

        else
            -- Get muscle parameters
            local muscle_params = params[muscle_name]
            
            -- Calculate left side activation
            local L_l = muscle_pair.left:normalized_fiber_length()
            local FMC_l = 0
            
            if (L_l - muscle_params.L0) >= 0 then
                FMC_l = muscle_params.c * normalized_GRF_l_y * (L_l - muscle_params.L0)
            end
            -- Special handling for glut_med
            -- if muscle_name == "glut_med" and normalized_GRF_r_y > 0 and calcn_r:com_pos().x > calcn_l:com_pos().x then
            --     FMC_l = 0
            -- end

            -- Calculate right side activation
            local L_r = muscle_pair.right:normalized_fiber_length()
            local FMC_r = 0
            
            if (L_r - muscle_params.L0) >= 0 then
                FMC_r = muscle_params.c * normalized_GRF_r_y * (L_r - muscle_params.L0)
            end
            -- Special handling for glut_med
            -- if muscle_name == "glut_med" and normalized_GRF_l_y > 0 and calcn_l:com_pos().x > calcn_r:com_pos().x then
            --     FMC_r = 0
            -- end
            
            -- Calculate final activations
            local activation_l = muscle_params.alpha * FMC_l
            local activation_r = muscle_params.alpha * FMC_r
            
            -- Store activations for data logging
            activations[muscle_name] = {
                left = activation_l,
                right = activation_r,
                FMC_l = FMC_l,
                FMC_r = FMC_r
            }
            
            -- Apply activations
            muscle_pair.left:add_input(activation_l)
            muscle_pair.right:add_input(activation_r)
        end
    end
    
    prev_grf_l = normalized_GRF_l_y
    prev_grf_r = normalized_GRF_r_y
    
    return false
end

function store_data( current_frame )
    -- Store FMC values for all muscles
    for muscle_name, activation_data in pairs(activations) do
        current_frame:set_value(muscle_name .. "_l.FMC", activation_data.left)
        current_frame:set_value(muscle_name .. "_r.FMC", activation_data.right)
    end
end