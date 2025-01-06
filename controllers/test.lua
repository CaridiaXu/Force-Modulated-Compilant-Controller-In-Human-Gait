function init( model, par, side )
    -- This function is called at the start of the simulation
    -- 'model' can be used to initialize the desired actuators (see LuaModel)
    -- 'par' can be used to define parameters for optimization (see LuaPar)
    -- 'side' denotes if the controller is for a specific side (-1 = left, 0 = unspecified, 1 = right)

    -- scone.debug("\nIf hamstrings_l-pelvis_tilt exists?")
    -- if model:has_custom_value("hamstrings_l-pelvis_tilt") then
    --     scone.debug("\nYES")
    -- else
    --     scone.debug("\nNO")
    -- end

    -- scone.debug("\nActuators:")
	-- for i = 1, model:actuator_count(), 1 do
	-- 	scone.debug(model:actuator(i):name())
	-- end

    -- scone.debug("\nDof:")
	-- for i = 1, model:dof_count(), 1 do
	-- 	scone.debug(model:dof(i):name())
	-- end

    -- scone.debug("\nMuscles:")
	-- for i = 1, model:muscle_count(), 1 do
	-- 	scone.debug(model:muscle(i):name())
	-- end

    -- scone.debug("\nBodys:")
	-- for i = 1, model:body_count(), 1 do
	-- 	scone.debug(model:body(i):name())
	-- end

    -- scone.debug("\nJoints:")
	-- for i = 1, model:joint_count(), 1 do
	-- 	scone.debug(model:joint(i):name())
	-- end

    -- scone.debug("\nLegs:")
	-- for i = 1, model:leg_count(), 1 do
	-- 	scone.debug(model:leg(i):name())
	-- end

    
    -- GRF = body:contact_force()
    -- GRF_y = GRF.y
    -- scone.debug("\nGRF in y direction:")
    -- scone.debug(GRF_y)
    -- mass = model:mass()
    -- gravity = model:gravity()
    -- gravity_y = gravity.y
    -- normalized_GRF = GRF_y / ( mass * -gravity_y )
    -- scone.debug("normalized GRF in y direction:")
    -- scone.debug(normalized_GRF)

    -- scone.debug("\nIf hamstrings_l-pelvis_tilt exists?")
    -- par:get("pelvis_tilt.offset")
    -- scone.debug(kp)

    -- hamstrings_left = model:find_muscle("hamstrings_l")
    -- L = hamstrings_left:muscle_tendon_length()
    -- scone.debug("\n muscle_tendon_length in hamstrings_l")
    -- scone.debug(L)
    -- V = hamstrings_left:muscle_tendon_velocity()
    -- scone.debug("\n muscle_tendon_velocity in hamstrings_l")
    -- scone.debug(V)
    
end

function update( model, time, controller )
    -- This function is called at each simulation timestep
    -- Use it to update the actuator inputs
    -- 'model' can be used to update the desired actuators (see LuaModel)
    -- 'time' is the time elapsed since this controller was activated
    -- 'controller' can be used to enable/disable child controllers of this ScriptController (see LuaController)
    
    body = model:find_body("calcn_l")
    pos = body:com_pos()
    pos_y = pos.y
    scone.debug("foot position in y direction:")
    scone.debug(pos_y)

    -- local t = model:time()
    
    return false -- change to 'return true' to terminate the simulation early
end

function store_data( current_frame )
    -- This function is called at each simulation timestep
    -- 'current_frame' can be used to store values for analysis (see LuaFrame)

    current_frame:set_value("hamstrings_l.muscle_tendon_length", L)
    current_frame:set_value("hamstrings_l.muscle_tendon_velocity", V)

end
