function nms = nonMaximumSurpression(matches)

% 

% Sort matching result.
[~,sorted_index] = sort(matches(:,5));

for i = 1 : size(matches,2)
    sorted_matches(:,i) = matches(sorted_index,i);
end

counter = 1;
size_sorted_matches = size(sorted_matches,1);
while size_sorted_matches > 0
    first = sorted_matches(1,:);
    nms(counter,:) = first;
    counter = counter + 1;
    sorted_matches(1,:) = [];
    size_sorted_matches = size_sorted_matches - 1;
    i = 1;
    while i <= size_sorted_matches
        overlap_left = max(first(1),sorted_matches(i,1));
        overlap_top = max(first(2),sorted_matches(i,2));
        overlap_right = min(first(1)+first(3),sorted_matches(i,1)+sorted_matches(i,3));
        overlap_bottom = min(first(2)+first(4),sorted_matches(i,2)+sorted_matches(i,4));
        overlap_width = overlap_right-overlap_left+1;
        overlap_height = overlap_bottom-overlap_top+1;
        if overlap_width > 0 && overlap_height > 0
            first_area = first(3)*first(4);
            second_area = sorted_matches(i,3)*sorted_matches(i,4);
            overlap_area = overlap_width*overlap_height/min(first_area,second_area);
            if overlap_area > 0.2
                sorted_matches(i,:) = [];
                size_sorted_matches = size_sorted_matches - 1;
            else
                i = i + 1;
            end
        else
            i = i + 1;
        end
    end
end







