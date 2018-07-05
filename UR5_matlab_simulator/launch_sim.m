function launch_sim(script_name, auto_launch, test_mode, sliding_mode)
link_2_gazebo = 1;
write_param_file(link_2_gazebo, auto_launch, test_mode, sliding_mode)
run(script_name)
end

