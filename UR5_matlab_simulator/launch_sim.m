function launch_sim(script_name, auto_launch)
param_file = fopen('param_script_sim.m','w');
fprintf(param_file,'link_2_gazebo = 1;\n');
fprintf(param_file,sprintf('auto_launch = %d;\n',auto_launch));
fclose(param_file);
run(script_name)
end

