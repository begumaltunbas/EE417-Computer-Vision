function [cars_array] = car_fun(img)


dil_strel=strel('disk',5); %dilation mask like what kind of thing we want to dilate
dilated=imdilate(img,dil_strel); % dilates the image, makes objects more visible and fills in small holes in objects. we obtain thicker lines and car objects
ero_strel=strel('disk',5); %erode mask like what kind of thing we want to erode/ suppress
eroded=imerode(dilated,ero_strel); % morphological erosion removes islands and small objects so that only substantive objects remain.
difference=imsubtract(dilated,eroded); % take difference of dilated and eroded image obtain a skeleton/edge.
difference=mat2gray(difference);

figure  (4)
imshow (difference) 

difference=conv2(difference,[1 1;1 1]); 
figure  (5)
imshow (difference)

difference=imadjust(difference,[0.4 0.9],[]);% map intensity values of 0.4-0.9 to 0-1
figure (6)
imshow (difference)

binary=logical(difference); % convert to binary, logical 0-1
[row, col]=size(binary);
figure(7)
imshow(binary)

fill_cars=imfill(binary,'holes'); %filling the object-fill holes/ car siluet and make it compact
figure(8)
imshow(fill_cars)

filter_lines=imerode(fill_cars,strel('line',8,55)); % erode the lines from the filtered binary image so that only cars remain in image - strel 8 length, 55 angle
figure(9)
imshow(filter_lines)

filter_lines=logical(filter_lines);

final_filtered= bwareafilt(filter_lines,[1000 10000000]); %to only obtain the car areas, we put threshold / avoid lines

figure(10)
imshow(final_filtered)

figure (11)
imshowpair(img,final_filtered); %put original and filtered image on eachother

final_bb=regionprops(final_filtered,'BoundingBox','Image'); %obtain the boundingboxes
cars_array = final_bb;
hold on
for n=2:size(final_bb,1)
rectangle('Position',final_bb(n).BoundingBox,'EdgeColor','w','LineWidth',2);
end
hold off




end

