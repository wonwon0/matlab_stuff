function write_param_file(link_2_gazebo, auto_launch, test_mode, sliding_mode)
param_file = fopen('param_script_sim.m','w');
fprintf(param_file,sprintf('link_2_gazebo = %d;\n',link_2_gazebo));
fprintf(param_file,sprintf('auto_launch = %d;\n',auto_launch));
fprintf(param_file,sprintf('test_mode = [%d, %d, %d];\n',test_mode, test_mode, test_mode));
fprintf(param_file,sprintf('sliding_mode = %d;\n',sliding_mode));
fclose(param_file);
end

