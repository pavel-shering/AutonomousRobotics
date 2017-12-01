function [padded_map] = pad_map(map, radius)
% figure; imshow(map);
se = strel('disk',radius);
padded_map = imdilate (map, se);
% figure('Name','PADDED'); imshow(padded_map);
end
