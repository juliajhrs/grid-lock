%% MTRN4230 - PROJECT 2
% Author: Julia Joharis
% zID   : z5383753
%% Resetting
clear; clc; close all;
%% MAIN DEMONSTRATION SPACE
close all; clc; startup_rvc;
%---------------------------------SETUP------------------------------------
% TCP Host and Port settings
% host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
% host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
vacuum_port = 63352;

% Calling the constructor of rtde and vacuum and cam to setup tcp connction
clear rtde;
rtde = rtde(host,port);
vacuum = vacuum(host,vacuum_port);
camList = webcamlist;
%--------------------------------------------------------------------------

% 1. TAKE A PICTURE
cam = webcam(2);
I = snapshot(cam);
imshow(I)
% I = imread("3.jpg"); % For use with sample images

% 2. SETUP GAME ENVIRONMENT
% [board_pt,tform,gameBoardCoordinates,tform_BirdsEye,gameBoard,blueCentroids,redCoordinates,blueCoordinates,greenCoordinates,moveToBorders,moveToRed,moveToBlue,moveToGreen] = setUpGameEnvironment(I);

% 3. VALIDATE BOARDGAME BORDER COORDINATES AND GAME PIECES COORDINATE
% moveToBorders;moveToRed;moveToBlue;moveToGreen; --> choose
% moveRobotToBoardPieces(rtde, moveToBorders, vacuum);

% % 4. RECEIVE OCCUPANCY GRID FROM MARKER
% partC_grid = [ 1 1 1 1 1 1 1 1 1 1
%                1 1 0 0 0 0 0 0 1 1
%                1 0 0 0 1 0 0 0 0 1
%                1 0 0 1 0 0 1 0 0 1
%                1 4 2 0 0 2 2 0 3 1
%                1 1 0 0 0 0 0 0 1 1
%                1 1 1 1 1 1 1 1 1 1 ];

% % 5. PART B DEMONSTRATION: GIVEN GRID, MOVE WITH BUG PATH
% partB_grid = partC_grid;
% [p,robot_bug_path,moveStartToGoal] = createBugPathForCorrectGrid(partB_grid, gameBoard, tform_BirdsEye, tform);
% moveWithBug(rtde, moveStartToGoal, vacuum);

% % 6. PART C DEMONSTRATION: FIX BLUE SQUARE IN THE WRONG SPOT
% [moveToFixBlue] = moveToFixBlueSquares(partC_grid,board_pt,blueCentroids,blueCoordinates,tform);
% moveToFixBluePieces(rtde, moveToFixBlue, vacuum);
% moveWithBug(rtde, moveStartToGoal, vacuum);

%% MAIN FUNCTIONS

% PART A: COMPUTER VISION
function [board_pt,tform,gameBoardCoordinates,tform_BirdsEye,gameBoard,blueCentroids,redCoordinates,blueCoordinates,greenCoordinates,moveToBorders,moveToRed,moveToBlue,moveToGreen] = setUpGameEnvironment(I)
    % Part A.1: Identifying Aruco Markers
    original = I;
    transformImg = I;
    
    [ids,locs,detectedFamily] = readArucoMarker(I, "DICT_4X4_250");
    numMarkers = length(ids);
    for i = 1:numMarkers
      loc = locs(:,:,i);
      % Display the marker ID and family
      disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))  
      % Insert marker edges
      I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
      % Insert marker corners
      markerRadius = 6;
      numCorners = size(loc,1);
      markerPosition = [loc,repmat(markerRadius,numCorners,1)];
      I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
      % Insert marker IDs
      center = mean(loc);
      I = insertText(I,center,ids(i),FontSize=30,BoxOpacity=1);
    end
    
    marked_img = I;
    
    % Part A.2: Getting Aruco Markers' Points in Image
    % Finding the centres of the marker
    % Table markers centre
    topLeftTable_pt = mean(locs(:, :, find(ids == 1)));
    topRightTable_pt = mean(locs(:, :, find(ids == 2)));
    bottomLeftTable_pt = mean(locs(:, :, find(ids == 3)));
    bottomRightTable_pt = mean(locs(:, :, find(ids == 4)));
    % Board markers centre
    topLeftBoard_pt = mean(locs(:, :, find(ids == 5)));
    topRightBoard_pt = mean(locs(:, :, find(ids == 6)));
    bottomLeftBoard_pt = mean(locs(:, :, find(ids == 7)));
    bottomRightBoard_pt = mean(locs(:, :, find(ids == 8)));
    
    % Defining the table and board coordinates as well as real world coordinates
    table_pt = [topLeftTable_pt; topRightTable_pt; bottomLeftTable_pt; bottomRightTable_pt];
    board_pt = [topLeftBoard_pt; topRightBoard_pt; bottomLeftBoard_pt; bottomRightBoard_pt];
    tableWorld = [-230,60; -230,-520; -990,60; -990,-520]; % Real-world coordinates of table markers 
    world = [50, 50; 340, 50; 50, 540; 340, 540];
    
    % Part A.3: Image Warping
    % Transform table and board to robots coordinate
    tform = fitgeotrans(table_pt, tableWorld, 'projective'); % Transformation matrix
    gameBoardCoordinates = transformPointsForward(tform, board_pt)
    
    % Gets birds eye view of board
    tform_BirdsEye = fitgeotrans(board_pt, world, 'projective');
    outputView = imref2d([590, 390]);
    gameBoard = imwarp(transformImg, tform_BirdsEye, "OutputView",outputView);
    
    % Display all images
    figure;
    tiledlayout(2,2);
    nexttile;
    imshow(original);
    title("Original Image");
    nexttile;
    imshow(marked_img);
    title("Detected Aruco Markers");
    nexttile;
    imshow(gameBoard);
    title("Transformed Image");
    
    % Part A.4: Color Masking for Location of Game Pieces
    % RGB masking range
    redMask = (gameBoard(:,:,1) >= 150) & (gameBoard(:,:,1) <= 255) & ...
              (gameBoard(:,:,2) >= 20) & (gameBoard(:,:,2) <= 50) & ...
              (gameBoard(:,:,3) >= 40) & (gameBoard(:,:,3) <= 70);
    blueMask = (gameBoard(:,:,1) >= 0) & (gameBoard(:,:,1) <= 70) & ...
               (gameBoard(:,:,2) >= 0) & (gameBoard(:,:,2) <= 70) & ...
               (gameBoard(:,:,3) >= 150) & (gameBoard(:,:,3) <= 255);
    greenMask = (gameBoard(:,:,1) >= 0) & (gameBoard(:,:,1) <= 70) & ...
               (gameBoard(:,:,2) >= 100) & (gameBoard(:,:,2) <= 255) & ...
               (gameBoard(:,:,3) >= 0) & (gameBoard(:,:,3) <= 70);
    
    % Morph operations to clean up masks
    se = strel('square', 8);
    redMask = imclose(redMask, se);
    blueMask = imclose(blueMask, se);
    greenMask = imclose(greenMask, se);
    redMask = bwareaopen(redMask, 100);
    blueMask = bwareaopen(blueMask, 100);
    greenMask = bwareaopen(greenMask, 100);
    
    % Display masks
    % figure;
    % tiledlayout(3,1);
    % nexttile;
    % imshow(redMask);
    % title("Red Mask on Image");
    % nexttile;
    % imshow(blueMask);
    % title("Blue Mask on Image");
    % nexttile;
    % imshow(greenMask);
    % title("Green Mask on Image");
    
    % Finding centres of masks
    redProps = regionprops(redMask, "Centroid");
    redCentroids = cat(1,redProps.Centroid);
    blueProps = regionprops(blueMask, "Centroid");
    blueCentroids = cat(1,blueProps.Centroid);
    greenProps = regionprops(greenMask, "Centroid");
    greenCentroids = cat(1,greenProps.Centroid);
    
    % Plotting cirles on each RGB squares
    figure;
    tiledlayout(1,1);
    nexttile;
    imshow(gameBoard);
    title("Centres of Game Squares");
    hold on;
    viscircles(redCentroids, 20, 'Color','m'); % magenta for reds
    viscircles(blueCentroids, 20, 'Color','c'); % cyan for blues
    viscircles(greenCentroids, 20, 'Color','y'); % yellow for greens
    plot(redCentroids(:,1), redCentroids(:,2), "m*")
    plot(blueCentroids(:,1), blueCentroids(:,2), "c*")
    plot(greenCentroids(:,1), greenCentroids(:,2), "y*")
    hold off;
    
    % Transform centroid coordinates into tform coordinates
    red = transformPointsInverse(tform_BirdsEye, redCentroids);
    redCoordinates = transformPointsForward(tform, red)
    blue = transformPointsInverse(tform_BirdsEye, blueCentroids);
    blueCoordinates = transformPointsForward(tform, blue)
    green = transformPointsInverse(tform_BirdsEye, greenCentroids);
    greenCoordinates = transformPointsForward(tform, green)

    % Move to every pieces to confirm coordinates
    figure;
    moveToBorders = createPathToGamePieces(gameBoardCoordinates, 1, 'Board');
    moveToRed = createPathToGamePieces(redCoordinates, 2, 'Red');
    moveToBlue = createPathToGamePieces(blueCoordinates, 3, 'Blue');
    moveToGreen = createPathToGamePieces(greenCoordinates, 4, 'Green');
end

% PART B: Grid Novices - Given a corrected occupancy grid, create a bug path from start to finish
function [p,robot_bug_path,moveStartToGoal] = createBugPathForCorrectGrid(partB_grid, gameBoard, tform_BirdsEye, tform)
    % Part B.1: Put occupancy map here
    % % Occupancy grid for 3.jpg
    % partB_grid = [ 1 1 1 1 1 1 1 1 1 1
    %                1 1 0 0 0 0 0 0 1 1
    %                1 0 0 0 1 0 0 0 0 1
    %                1 0 0 1 0 0 1 0 0 1
    %                1 4 2 0 0 2 2 0 3 1
    %                1 1 0 0 0 0 0 0 1 1
    %                1 1 1 1 1 1 1 1 1 1 ];
    
    % % Occupancy grid for 4.jpg
    % partB_grid = [ 1 1 1 1 1 1 1 1 1 1
    %                1 1 0 0 0 0 1 1 1 1
    %                1 0 1 0 1 0 0 2 0 1
    %                1 0 2 1 2 0 1 0 0 1
    %                1 0 2 4 1 2 2 0 3 1
    %                1 1 0 0 0 0 0 0 1 1
    %                1 1 1 1 1 1 1 1 1 1 ];
    
    % % Occupancy grid for 5.jpg
    % partB_grid = [ 1 1 1 1 1 1 1 1 1 1
    %                1 1 4 0 0 0 0 1 1 1
    %                1 2 1 0 1 0 0 0 0 1
    %                1 1 2 1 2 0 1 0 0 1
    %                1 1 2 0 0 0 2 0 3 1
    %                1 1 0 0 0 0 0 0 1 1
    %                1 1 1 1 1 1 1 1 1 1 ];
 
    % Part B.2: Grid re-sizing and axis correction
    scale_fact = 10; % scale up grid
    [y,x] = size(partB_grid);
    grid = imresize(partB_grid, [y*scale_fact x*scale_fact], 'nearest');
    original_grid = grid;
    grid = flipud(grid); % flip up-down to fix origin placement
    
    % Part B.3: Identifying start and destination points
    % Defining grid parameters
    destination = 4;
    player = 3;
    % Find player & destination (and blue squares) from scaled grid 
    [x_startcoord, y_startcoord] = find(grid == player);
    [x_destcoord, y_destcoord] = find(grid == destination);
    x_start = floor(median(x_startcoord));
    y_start = floor(median(y_startcoord));
    x_dest = floor(median(x_destcoord));
    y_dest = floor(median(y_destcoord));
    
    % Part B.4: Redefining Player/Destination as 'Clear' & Obstacles Offset
    % B.4.1: Redefine player & destination grids as 0 for 'clear'
    grid_p = grid;
    grid_p(x_startcoord, y_startcoord) = 0;
    grid_p(x_destcoord, y_destcoord) = 0;
    
    % B.4.2: Obstacle offsetting for Linear Path Generation
    path_gap = 1;
    path_tightening_coeff = 2;
    offset = (scale_fact/2)-path_gap;
    se = strel('square', offset*path_tightening_coeff); % structuring element for dilation of boundary
    grid_offset = grid_p;
    grid_offset = imdilate(grid_offset, se);
    grid_offset = grid_offset(1:size(grid_p,1), 1:size(grid_p,2)); % cropping to keep a fixed size
    for i = 1:4
        % clearing a narrow spot at start
        grid_offset(x_start, y_start+i) = 0;
        grid_offset(x_start+i, y_start) = 0;
        grid_offset(x_start, y_start-i) = 0;
        grid_offset(x_start-i, y_start) = 0;
        % same but at destination
        grid_offset(x_dest, y_dest+i) = 0;
        grid_offset(x_dest+i, y_dest) = 0;
        grid_offset(x_dest, y_dest-i) = 0;
        grid_offset(x_dest-i, y_dest) = 0;
    end
    
    % Display all images
    figure;
    tiledlayout(2,2);
    nexttile;
    imshow(original_grid);
    title("Original Map with Origin at Top-Left");
    nexttile;
    imshow(grid);
    title("Origin at Bottom-Left");
    nexttile;
    imshow(grid_p);
    title("Cleared Start & Goal");
    nexttile;
    imshow(grid_offset);
    title("Added Offsets");
    
    % Part B.5: Bug Algorithm Generation on Offsetted Map
    bug = Bug2(grid_offset);
    figure;
    bug.plot();
    src = [y_start, x_start];
    dest = [y_dest, x_dest];
    p = bug.query(src, dest, 'animate');
    
    % B.5.1: Plotting bug path on grid map
    grid_bug = grid_p;
    grid_bug = insertShape(grid_bug,"circle",[y_start x_start 1],LineWidth=2,ShapeColor="green",Opacity=1); % start
    grid_bug = insertShape(grid_bug,"circle",[y_dest x_dest 1],LineWidth=2,ShapeColor="blue",Opacity=1); % destination
    figure;
    tiledlayout(1,1);
    nexttile;
    imshow(grid_bug);
    title("Bug Path on Cleared Start/Goal Grid");
    xlabel('y-axis (100mm)');
    ylabel('x-axis (70mm)');
    set(gca, 'YDir', 'normal'); % fix y-direction
    hold on;
    plot(p(:,1), p(:,2), 'r', 'LineWidth', 3); % flip y and x if map is oriented differently
    hold off;
    
    % Part B.6: Generating Robot Trajectory For Obtained Bug Path
    % B.6.1: Filtering path movements
    p_filtered = p(1,:);
    for i = 2:size(p,1)
        if ~isequal(floor(p(i,:)/10), floor(p_filtered(end,:)/10))
            p_filtered = [p_filtered; p(i,:)];
        end
    end
    p_filtered = floor(p_filtered/10);
    p = p_filtered;
    
    % B.6.2: Applying 2D linear transformation to map bug path coordinates to image coordinates
    % Tuning parameters:-
    % SCALING: shrinks(--)/stretch(++)
    yscale_coeff = -10; % stretches vertically
    xscale_coeff = -7.5; % stretches sideways
    % TRANS: (++)/(--)
    ytransl_coeff = 87; % up/down translation
    xtransl_coeff = 41.5; % left/right translation
    
    tform_bug_matrix = [y*yscale_coeff 0 y*ytransl_coeff; 0 x*xscale_coeff x*xtransl_coeff; 0 0 1]; 
    tform_bug = affinetform2d(tform_bug_matrix);
    p = transformPointsForward(tform_bug, p); % obtains bug path in image coordinates
    
    % B.6.3: Plotting bug path image coordinate on game board
    % Define start & goal bug-image coordinates
    p = flip(p,2);
    [r,~] = size(p);
    start = p(1,:);
    goal = p(r,:);
    
    figure;
    tiledlayout(1,1);
    nexttile;
    imshow(gameBoard);
    title("Bug Path on Sample Image");
    xlabel('x-axis (7)');
    ylabel('y-axis (10)');
    hold on;
    plot(start(:,1), start(:,2), "g*");
    plot(goal(:,1), goal(:,2), "r*");
    plot(p(:,1), p(:,2), 'r', 'LineWidth', 1);
    hold off;
    
    % B.6.4: Converting bug-image coordinates to robot coordinates
    inv_p = transformPointsInverse(tform_BirdsEye, p);
    robot_bug_path = transformPointsForward(tform, inv_p);
    
    % Generate robot movement
    figure;
    moveStartToGoal = createBugPath(robot_bug_path, 1, 'Bug: Start to Goal');
end

% PART C: GRID MASTERS - Fixing blue squares positions before using bug path
function [moveToFixBlue] = moveToFixBlueSquares(partC_grid,board_pt,blueCentroids,blueCoordinates,tform)
    % Defining grid parameters
    blues = 2;
    
    % Part C.2: Matching Orientation of Grid to Board Game
    % C.2.1: Creating geometric transformation from matching control points
    [gr,gc] = size(partC_grid);
    arucoid5_Match = [1+1,gc-1];
    arucoid6_Match = [gr-1,gc-1];
    arucoid7_Match = [1+1,1+1];
    arucoid8_Match = [gr-1,1+1];
    movingPoints = flip([arucoid5_Match; arucoid6_Match; arucoid7_Match; arucoid8_Match],2);
    fixedPoints = board_pt;
    tform_GridToBoard = fitgeotform2d(movingPoints, fixedPoints,"projective");
    gridC_AlignedToBoard = imwarp(partC_grid,tform_GridToBoard, 'InterpolationMethod', 'nearest');
    
    % Part C.3: Identifying Blue Square in the Wrong Place
    blueFromBoard = transformPointsInverse(tform_GridToBoard,blueCentroids);
    blueFromBoard = flip(blueFromBoard,2); % flip back row/col to match grid coords
    blue_WrongPos = [];
    [bx,~] = size(blueFromBoard);
    for i = 1:bx
        if (partC_grid(blueFromBoard(i,:))~=blues) % found blue square on the wrong spot
            blue_WrongPos = blueCoordinates(i,:); % store its robot coord
        end
    end
    
    % Part C.4: Pick and Place Blue Square to the Correct Spot
    % Finding the correct spot
    blueOnGrid = find(partC_grid == blues);
    blue_CorrectPos = [];
    [ab_x,~] = size(blueOnGrid);
    for i = 1:ab_x
        for j = 1:bx
            if (blueOnGrid(i,:) ~= blueFromBoard(j,:)) % found the correct spot
                blue_CorrectPos = blueOnGrid(i,:);
            end
        end
    end
    
    % Transform correct position into robot's coords
    blue_CorrectPos = transformPointsForward(tform_GridToBoard,blue_CorrectPos);
    blue_CorrectPosRobot = transformPointsForward(tform, blue_CorrectPos);
    
    % Creating trajectory to fix blue's position
    blue_moveToFix = [blue_WrongPos; blue_CorrectPosRobot];
    moveToFixBlue = createPathToFixBlue(blue_moveToFix, 1, 'Path To Fix Blue Square Position');
    
    % % Display all images
    % figure;
    % tiledlayout(1,3);
    % nexttile;
    % imshow(partC_grid);
    % title("Original Map");
    % nexttile;
    % imshow(gridC_AlignedToBoard);
    % title("Map Aligned to Board");
    % nexttile;
    % imshow(original);
    % title("Map IRL");
end

%% HELPER FUNCTIONS
% Function to generate trajectory to move to every game pieces in part A
function traj = createPathToGamePieces(coordinatesArray, count, titled)
    clear traj % Reset trajectory array before execution
    traj = [];
    liftHeight = 0.5 * 0.1 * 1000;
    [rows, ~] = size(coordinatesArray);
    for i = 1:rows
        traj = [traj; coordinatesArray(i,:), 0];
        traj = [traj; coordinatesArray(i,:), liftHeight];
    end
    traj = [traj; -588.53, -133.30, 371.91]; % go back to home once finished

    subplot(2,2,count)
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    plot3(traj(:,1), traj(:,2), traj(:,3));
    title(titled)
end

% Function to generate bug path trajectory in part B
function traj = createBugPath(pathCoordinates, count, titled)
    clear traj % Reset trajectory array before execution
    traj = [];
    [rows, ~] = size(pathCoordinates);
    liftHeight = 0.5 * 0.1 * 1000;
    traj = [traj; pathCoordinates(1,:), 0]; % picks up player piece
    for i = 1:rows
        traj = [traj; pathCoordinates(i,:), liftHeight];
    end
    traj = [traj; pathCoordinates(rows,:), 0]; % places once reaches destination
    traj = [traj; -588.53, -133.30, 371.91]; % go back to home position

    subplot(1,1,count)
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    plot3(traj(:,1), traj(:,2), traj(:,3));
    title(titled)
end

% Function to generate bug path trajectory in part B
function traj = createPathToFixBlue(pathCoordinates, count, titled)
    clear traj % Reset trajectory array before execution
    traj = [];
    [rows, ~] = size(pathCoordinates);
    liftHeight = 0.5 * 0.1 * 1000;
    traj = [traj; pathCoordinates(1,:), 0]; % picks up blue in wrong spot
    traj = [traj; pathCoordinates(1,:), liftHeight];
    traj = [traj; pathCoordinates(rows,:), 0]; % places in the right spot
    traj = [traj; pathCoordinates(rows,:), liftHeight];
    traj = [traj; -588.53, -133.30, 371.91]; % go back to home position

    subplot(1,1,count)
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    plot3(traj(:,1), traj(:,2), traj(:,3));
    title(titled)
end

% Function to move robot given a position
function moveRobotToDestination(rtde, pos, isFirstPos, vacuum)
    height = 20;
    pickup = 4.5;
    posWithHeight = [pos(1),pos(2),height,0,3.14,0];
    rtde.movel(posWithHeight, 'pose');

    if isFirstPos
        % apply vacuum if its first position
        pickupPos = [pos(1),pos(2),pickup,0,3.14,0];
        rtde.movel(pickupPos,'pose');
        vacuum.grip();
        pause(1);
        % go back up
        rtde.movel(posWithHeight,'pose');
    end
end

% Function to move robot to gameboard borders and game pieces
function moveRobotToBoardPieces(rtde, pos, vacuum)
    for i = 1:size(pos,1)
        % move without vacuuming
        moveRobotToDestination(rtde, pos(i,:), vacuum, false);
        pause(0.5);
    end
end

% Function to move robot with bug trajectory
function moveWithBug(rtde, pos, vacuum)
    for i = 1:size(pos,1)
        if i == 1
            % pick up game piece using vacuum
            moveRobotToDestination(rtde, pos(i,:), vacuum, true);
        else
            % if not first, don't let piece fall
            moveRobotToDestination(rtde, pos(i,:), vacuum, false);
        end
        pause(0.5);
    end
    % release vacuum at the end
    destination = pos(size(pos,1),:);
    finalPos = [destination(1),destination(2),4.5,0,3.14,0];
    rtde.movel(finalPos,'pose');
    vacuum.release();
    pause(1);
end

% Function to move to fix blue squares
function moveToFixBluePieces(rtde, pos, vacuum)
    % picks up blue square
    moveRobotToDestination(rtde, pos(1,:), vacuum, true);
    pause(0.5);
    
    % places blue and release vacuum at the end
    destination = pos(size(pos,1),:);
    finalPos = [destination(1),destination(2),4.5,0,3.14,0];
    rtde.movel(finalPos,'pose');
    vacuum.release();
    pause(1);
end