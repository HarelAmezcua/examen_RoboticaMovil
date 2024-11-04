% detectAprilTag.m
function [id, uv, Rt_pose] = detectAprilTag(image, K, tagSize)
    [id, uv, Rt_pose] = readAprilTag(image, "tag36h11", K, tagSize);
end
