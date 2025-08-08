function [clear_flag, dist] = isPathClear(p1, p2, obs_center, obs_radius)
% isPathClear - Checks if the line segment from p1 to p2 intersects a circle.
%
% The underlying geometry of this function is to find the minimum distance
% between a point (the obstacle's center) and a line segment (the robot's path).
%
% INPUTS:
%   p1         - Start point of the segment [x1; y1] (e.g., robot's current position)
%   p2         - End point of the segment [x2; y2] (e.g., the waypoint/goal)
%   obs_center - Center of the obstacle [xc; yc]
%   obs_radius - The safety radius of the obstacle (physical radius + safety margin)
%
% OUTPUTS:
%   clear_flag - A boolean: true if the path is clear, false if a collision is imminent.
%   dist       - The calculated minimum distance from the path to the obstacle's center.

    %% STEP 1: Define the vectors for the geometric problem

    % The vector 'v' represents the robot's path segment. It starts at p1 and points to p2.
    % This vector defines the direction and length of the line segment.
    v = p2 - p1;
    
    % The vector 'w' goes from the robot's starting point (p1) to the obstacle's center.
    % This vector will be used to find the projection of the obstacle onto the path's line.
    w = obs_center - p1;

    %% STEP 2: Project the obstacle onto the infinite line defined by the path

    % The scalar projection of vector 'w' onto vector 'v' tells us how far along 'v'
    % we must travel from p1 to find the point on the line closest to obs_center.
    % The formula for the normalized scalar projection is t_proj = (w . v) / |v|^2.
    % This 't_proj' is a ratio.
    t_proj = dot(w, v) / dot(v, v);
    
    % GEOMETRIC INTERPRETATION of 't_proj':
    % - If 0 < t_proj < 1, the closest point on the infinite line lies WITHIN the segment p1-p2.
    % - If t_proj <= 0, the closest point is behind p1 (or is p1 itself).
    % - If t_proj >= 1, the closest point is ahead of p2 (or is p2 itself).

    %% STEP 3: Clamp the projection to ensure it lies ON the segment
    
    % Since the robot only travels on the segment p1-p2, we are not interested in points
    % on the infinite line that are outside this segment. We "clamp" the value of 't_proj'
    % to the interval [0, 1]. This is the key step to correctly handle cases where the
    % obstacle is "behind" the start or "ahead" of the end of the path.
    t_clamped = max(0, min(1, t_proj));
    
    %% STEP 4: Calculate the coordinates of the closest point on the segment
    
    % Using the clamped ratio 't_clamped', we can now find the exact coordinates of the
    % closest point. This is a simple linear interpolation: we start at p1 and move
    % along the direction of vector 'v' by a fraction 't_clamped' of its length.
    closest_point_on_segment = p1 + t_clamped * v;
    
    %% STEP 5: Calculate the final distance and make the decision
    
    % Finally, we compute the Euclidean distance between the obstacle's center and
    % the closest point we found on the robot's path.
    dist = norm(closest_point_on_segment - obs_center);
    
    % The path is considered "clear" if this minimum distance is greater than
    % the obstacle's safety radius.
    clear_flag = (dist > obs_radius);
    
end