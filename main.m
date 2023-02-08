close all ; clc ; clear ;

original = imread('above2.jpg');
gray=rgb2gray(original);

gauss=imgaussfilt(gray);

imshow(gauss);
h = imrect; %%crop a rectangle manually
position = getPosition(h);
crop = imcrop(gauss, position);
crop_gray=crop;


%%%%% IF USING CANNY (use above2.jpg or park2.jpeg) %%%%%
%image_thresholded= edge(I,'Sobel');
image_thresholded= edge(crop_gray,'Canny');
%%%%%%%%END OF CANNY %%%%%%%%%%%%%%%%


%%%%%%IF USING THE THRESHOLDER TOOL UNCOMMENT BELOW/COMMENT CANNY ABOVE (use temp22.jpeg)%%%%%%%%%
%Convert RGB image to chosen color space
% hsv = rgb2hsv(crop);
%  
% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.008;
% channel1Max = 0.456;
%  
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.000;
% channel2Max = 0.189;
%  
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.000;
% channel3Max = 1.000;
%  
% % Create mask based on chosen histogram thresholds
% sliderBW = (hsv(:,:,1) >= channel1Min ) & (hsv(:,:,1) <= channel1Max) & ...
%     (hsv(:,:,2) >= channel2Min ) & (hsv(:,:,2) <= channel2Max) & ...
%     (hsv(:,:,3) >= channel3Min ) & (hsv(:,:,3) <= channel3Max);
% BW = sliderBW;
% image_thresholded=BW;
%%%%%%%%%%%%%%%%%END OF TOOL%%%%%%%%%%%%%%%%%%%%%%

start_angle = -90;
end_angle = 89;
theta_resolution = 0.5 ;

[H,T,R] = hough(image_thresholded, 'Theta', start_angle:theta_resolution:end_angle);

peaks= houghpeaks(H,40,'threshold',ceil(0.2*max(H(:))));

lines = houghlines(image_thresholded,T,R,peaks,'FillGap',5,'MinLength',60);
figure(20), imshow(crop), hold on

max_len = 0;
min_len=10000000000;

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
   
   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
   if ( len < min_len)
      min_len = len;
      xy_short = xy;
   end
  
end
hold off


a=[];

for i=1:length(lines)
   slope = (lines(i).point2(2)-lines(i).point1(2))/(lines(i).point2(1)-lines(i).point1(1)) ;
   if abs(slope) > 0.05 
        a=[a;lines(i).point1 lines(i).point2 (lines(i).point1+lines(i).point2)/2];
   end
end

a=sortrows(a,1);

temp=a;

b=[];
b=[b;a(1,1) a(1,2) a(1,3) a(1,4) a(1,5) a(1,6)];

temp(1,1) = 100000;
temp(1,2) = 100000;
temp(1,3) = 100000;
temp(1,4) = 100000;
temp(1,5) = 100000;
temp(1,6) = 100000;

[row_a,col_a,ch]=size(a);
for i=1:row_a
   
    min_diff=10000000000000;
    
    [row_t,col_t,ch]=size(temp);
    
    for j=1:1:row_t
        x1=b(i,5);
        y1=b(i,6);
        x2=temp(j,5);
        y2=temp(j,6);
        p1=[x1 y1];
        p2=[x2 y2];
        diff= abs(norm(p1-p2));
        if diff < min_diff
            min_diff=diff;
            k=j;
        end
        
    end
    
    if ~isnan(k)
        b=[b;temp(k,1) temp(k,2) temp(k,3) temp(k,4) temp(k,5) temp(k,6)];
        temp(k,1) = 10000;
        temp(k,2) = 10000;
        temp(k,3) = 10000;
        temp(k,4) = 10000;
        temp(k,5) = 10000;
        temp(k,6) = 10000;
    end
     
   
 
end

b(end,:) = []; %Delete last row

a = b;
corners = detectHarrisFeatures(crop_gray);
hold off
figure(30);
imshow(crop); hold on;
plot(corners.selectStrongest(100));

%IF NUMBER OF LINES IS HIGHER THAN CORNER THRESHOLD SHOULD BE LESS
corners = corners.selectStrongest(row_a*8);
corners.Location=sortrows(corners.Location,1);



 %%%  CAR DETECTION ALGORITHM IS CALLED %%%
 cars_detected = car_fun(crop_gray);
 centroids = [];
 
 for i=1:length(cars_detected)
     
     x =(cars_detected(i).BoundingBox(1) + cars_detected(i).BoundingBox(3)/2 ) ;
     y =(cars_detected(i).BoundingBox(2) + cars_detected(i).BoundingBox(4)/2 ) ;
     
     centroids = [centroids ; [x y] ]; %store the center coord of detected cars
     
 end

 
x_object_list = centroids(:, 1);
y_object_list = centroids(:, 2);

figure(18)
imshow(crop);
hold on;
plot(x_object_list,y_object_list , 'r*', 'LineWidth', 2, 'MarkerSize', 15);


 %%%   %%% %%% %%%   %%%
    

figure(12);
imshow(crop);

mid_patch=[];
empty_counter=0;
max_corner_threshold = 10;
min_corner_threshold = 3;
for i=1:row_a-1  
    
    corners_in_patch(i) = 0;
    for j=1:length(corners) % each corner
        x=corners.Location(j,1);
        x=double(x);
        y=corners.Location(j,2);
        y=double(y);
        
        A = [ a(i,1) a(i+1,1) a(i,3) a(i+1,3)  ];
        B = [ a(i+1,1) a(i,1) a(i,3) a(i+1,3)  ];
        C = [ a(i,2) a(i+1,2) a(i+1,4) a(i,4)  ];
        D = [ a(i+1,4) a(i,4) a(i,2) a(i+1,2)  ];
        
        if         (x> min(A) ) ...
                && (x< max(B) ) ...
                && (y> min(C) ) ...
                && (y< max(D) )  %check if corner is inside the patch
            corners_in_patch(i) = corners_in_patch(i) + 1 ;
        end
       
    end

    %%OBJECT DETECTION METRIC
    object_in_patch(i) = 0;
    for k=1:length(x_object_list) % each object
        x_obj=x_object_list(k);
        y_obj=y_object_list(k);
        A = [ a(i,1) a(i+1,1) a(i,3) a(i+1,3)  ];
        B = [ a(i+1,1) a(i,1) a(i,3) a(i+1,3)  ];
        C = [ a(i,2) a(i+1,2) a(i+1,4) a(i,4)  ];
        D = [ a(i+1,4) a(i,4) a(i,2) a(i+1,2)  ];
        
        if (x_obj> min(A) ) ...
            && (x_obj< max(B) ) ...
            && (y_obj> min(C) ) ...
            && (y_obj< max(D) )  %check if there is car  inside the patch
            object_in_patch(i) = 1 ;
            corners_in_patch(i) = corners_in_patch(i) + 5 ; %add 5 to corners i.e 5 corners equivalent to car object
            
        end
       
    end
    
 
    %IF DISTANCE IS SO HIGH BETWEEN TWO LINES DO NOT PATCH!! - set a threshold    
    p1 = [a(i,5) a(i,6)];
    p2 = [a(i+1,5) a(i+1,6)];
    diff= abs(norm(p1-p2));
    
    if diff < 120
       
        if  corners_in_patch(i)>max_corner_threshold && object_in_patch(i)==1
            patch([a(i,1) a(i+1,1) a(i+1,3) a(i,3)], [a(i,2) a(i+1,2) a(i+1,4) a(i,4)], 'red');  
            alpha(0.4);
            empty_prob(i)=0;
        
        
        elseif  corners_in_patch(i)<max_corner_threshold && corners_in_patch(i)>min_corner_threshold 
            empty_prob(i)= 1-(corners_in_patch(i)/ max_corner_threshold);
            if empty_prob(i)<0.5 
                patch([a(i,1) a(i+1,1) a(i+1,3) a(i,3)], [a(i,2) a(i+1,2) a(i+1,4) a(i,4)], 'red');
                alpha(0.4);
            else
                patch([a(i,1) a(i+1,1) a(i+1,3) a(i,3)], [a(i,2) a(i+1,2) a(i+1,4) a(i,4)], 'yellow');
                alpha(0.4);
            end
           

        else 
            patch([a(i,1) a(i+1,1) a(i+1,3) a(i,3)], [a(i,2) a(i+1,2) a(i+1,4) a(i,4)], 'green');
            alpha(0.4);
            empty_counter=empty_counter+1;
            empty_prob(i)=1;
        end
        

    end
 
    
end
   hold on
   text(10,10,strcat('\color{black}#Free:',num2str(empty_counter)))
   hold on
   hold off
      
for i=1:row_a-1
    p1 = [a(i,5) a(i,6)];
    p2 = [a(i+1,5) a(i+1,6)];
    diff= abs(norm(p1-p2));
    if diff < 120
        mid_patch=[mid_patch;(a(i,5)+a(i+1,5))/2  (a(i,6)+a(i+1,6))/2  empty_prob(i)  corners_in_patch(i) object_in_patch(i) ];
   end
end

x_mid_patch = mid_patch(:, 1);
y_mid_patch = mid_patch(:, 2);
empty_mid_patch = mid_patch(:, 3);
for i=1:length(x_mid_patch) 
        text(x_mid_patch(i),y_mid_patch(i),strcat('\color{black}',num2str(round(empty_mid_patch(i),1))))
end
   




