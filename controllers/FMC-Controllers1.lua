function init( model, par, side )
    -- This function is called at the start of the simulation
    -- 'model' can be used to initialize the desired actuators (see LuaModel)
    -- 'par' can be used to define parameters for optimization (see LuaPar)
    -- 'side' denotes if the controller is for a specific side (-1 = left, 0 = unspecified, 1 = right)
    
    -- Define muscle pairs and their default L0 values
    -- muscle_configs = {
    --     {name = "rect_fem", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "add_mag", L0 = 0.11, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "glut_med", L0 = 0.14, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "hamstrings", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
    --     {name = "iliopsoas", L0 = 0.26, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0}
    -- }
    
    muscle_configs = {
        -- {name = "rect_fem", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
        -- {name = "add_mag", L0 = 0.11, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
        -- {name = "glut_med", L0 = 0.14, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
        {name = "hamstrings", L0 = 0.4, alpha = 5.0, std = 0.01, min = 0.0, max = 1.0},
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
        local default_L0 = config.L0
        
        -- Store muscle objects
        muscles[muscle_name] = {
            left = model:find_muscle(muscle_name .. "_l"),
            right = model:find_muscle(muscle_name .. "_r")
        }
        
        -- Create single parameter set for each muscle
        params[muscle_name] = {
            c = par:create_from_mean_std(muscle_name .. ".c", 0.26, 0.01, 0.0, 1.0),
            L0 = par:create_from_mean_std(muscle_name .. ".L0", default_L0, 0.01, 0.0, 1.0),
            alpha = config.alpha 
        }
    end
    
    -- KL_r = par:create_from_mean_std( "hamstrings_r.KL", 1.0, 0.01, 0.0, 10.0)
    -- KL = par:create_from_mean_std( "hamstrings.KL", 1.0, 0.01, 0.0, 2.0)
    -- KV = par:create_from_mean_std( "hamstrings.KV", 0.21, 0.002, 0.0, 2.0)

    -- Global parameters
    mass = model:mass()
    -- alpha = par:create_from_mean_std("alpha", 5.0, 0.5, 0.0, 10.0)
    -- alpha = 5.0 / mass
    -- alpha = 1.0
    
    gravity = model:gravity()
    gravity_y = gravity.y
end

function update( model, time, controller )
    -- This function is called at each simulation timestep
    -- Use it to update the actuator inputs
    -- 'model' can be used to update the desired actuators (see LuaModel)
    -- 'time' is the time elapsed since this controller was activated
    -- 'controller' can be used to enable/disable child controllers of this ScriptController (see LuaController)
    
    -- Get ground reaction forces
    local GRF_l = calcn_l:contact_force()
    local GRF_l_y = GRF_l.y
    local normalized_GRF_l_y = GRF_l_y / ( -gravity_y * mass )
    
    local GRF_r = calcn_r:contact_force()
    local GRF_r_y = GRF_r.y
    local normalized_GRF_r_y = GRF_r_y / ( -gravity_y * mass )
    
    -- Calculate and apply activations for each muscle
    activations = {}
    for muscle_name, muscle_pair in pairs(muscles) do
        -- Get muscle parameters
        local muscle_params = params[muscle_name]
        
        -- Calculate left side
        local L_l = muscle_pair.left:tendon_length() + muscle_pair.left:fiber_length()
        local FMC_l = muscle_params.c * normalized_GRF_l_y * ( L_l - muscle_params.L0 )
        local activation_l = muscle_params.alpha * FMC_l
        
        -- Calculate right side
        local L_r = muscle_pair.right:tendon_length() + muscle_pair.right:fiber_length()
        local FMC_r = muscle_params.c * normalized_GRF_r_y * ( L_r - muscle_params.L0 )
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
    
    -- RL_l = KL * ( muscles["hamstrings"].left:tendon_length() + muscles["hamstrings"].left:fiber_length() - params["hamstrings"].L0 )
    -- RL_r = KL * ( muscles["hamstrings"].right:tendon_length() + muscles["hamstrings"].right:fiber_length() - params["hamstrings"].L0 )
    -- RV_l = KV * ( muscles["hamstrings"].left:normalized_fiber_velocity() )
    -- RV_r = KV * ( muscles["hamstrings"].right:normalized_fiber_velocity() )
    
    -- muscles["hamstrings"].left:add_input( RL_l )
    -- muscles["hamstrings"].right:add_input( RL_r )
    -- muscles["hamstrings"].left:add_input( RV_l )
    -- muscles["hamstrings"].right:add_input( RV_r )
    
    return false -- change to 'return true' to terminate the simulation early
end

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