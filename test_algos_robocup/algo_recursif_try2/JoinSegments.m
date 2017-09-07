function [ trajectory ] = JoinSegments( trajectory_1, trajectory_2 )

    trajectory.points = [trajectory_1.points; trajectory_2.points(2:end,:)];
    trajectory_1.points;
    trajectory_2.points;
    trajectory.points;
    trajectory.start = trajectory.points(1,:);
    trajectory.goal = trajectory.points(end,:);
end

