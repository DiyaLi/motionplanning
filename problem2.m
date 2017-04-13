clear 
clc

% load inputs and coordinates
fileID1 = fopen('input_1.txt'); input1 = fscanf(fileID1,'%f',[3,inf])';
tot_vert(1) = input1(1,1); start(1) = input1(1,2); goal(1) = input1(1,3);
data.A.from = input1(2:length(input1),1);
data.A.to = input1(2:length(input1),2);
data.A.cost = input1(2:length(input1),3);

fileID2 = fopen('input_2.txt'); input2 = fscanf(fileID2,'%f',[3,inf])';
tot_vert(2) = input2(1,1); start(2) = input2(1,2); goal(2) = input2(1,3);
data.B.from = input2(2:length(input2),1);
data.B.to = input2(2:length(input2),2);
data.B.cost = input2(2:length(input2),3);

fileID3 = fopen('input_3.txt'); input3 = fscanf(fileID3,'%f',[3,inf])';
tot_vert(3) = input3(1,1); start(3) = input3(1,2); goal(3) = input3(1,3);
data.C.from = input3(2:length(input3),1);
data.C.to = input3(2:length(input3),2);
data.C.cost = input3(2:length(input3),3);

fileID4 = fopen('coords_1.txt');
coords.A = fscanf(fileID4,'%f',[2,inf])';

fileID5 = fopen('coords_2.txt');
coords.B = fscanf(fileID5,'%f',[2,inf])';

fileID6 = fopen('coords_3.txt');
coords.C = fscanf(fileID6,'%f',[2,inf])';

% output matrix
output_cost = zeros(3,6); output_numiter = zeros(3,6);
% Dijkstra
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64)));
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        % remove vertex with the lowest cost from open to close
        [val,node] = min(V(Open));
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,1) = V(goal(resol - 64));
    output_numiter(resol-64,1) = length(Close);
end

% A* with Euclidean distance as heuristic
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64)));
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        Vtemp = 0;
        for f = 1:length(Open)
            Vtest = V(Open(f));
            h = norm(coords.(char(resol))(Open(f),:)-coords.(char(resol))(goal(resol-64),:));
            Vtemp(f) = Vtest + h;
        end
        [val,node] = min(Vtemp);
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,2) = V(goal(resol - 64));
    output_numiter(resol-64,2) = length(Close);
end

% Weighted A* with e=2
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64))); e = 2;
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        Vtemp = 0;
        for f = 1:length(Open)
            Vtest = V(Open(f));
            h = norm(coords.(char(resol))(Open(f),:)-coords.(char(resol))(goal(resol-64),:));
            Vtemp(f) = Vtest + e*h;
        end 
        
        [val,node] = min(Vtemp);
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,3) = V(goal(resol - 64));
    output_numiter(resol-64,3) = length(Close);
end

% Weighted A* with e=3
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64))); e = 3;
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        Vtemp = 0;
        for f = 1:length(Open)
            Vtest = V(Open(f));
            h = norm(coords.(char(resol))(Open(f),:)-coords.(char(resol))(goal(resol-64),:));
            Vtemp(f) = Vtest + e*h;
        end
        % remove vertex with the lowest cost from open to close
        [val,node] = min(Vtemp);
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,4) = V(goal(resol - 64));
    output_numiter(resol-64,4) = length(Close);
end

% Weighted A* with e=4
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64))); e = 4;
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        Vtemp = 0;
        for f = 1:length(Open)
            Vtest = V(Open(f));
            h = norm(coords.(char(resol))(Open(f),:)-coords.(char(resol))(goal(resol-64),:));
            Vtemp(f) = Vtest + e*h;
        end
        % remove vertex with the lowest cost from open to close
        [val,node] = min(Vtemp);
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,5) = V(goal(resol - 64));
    output_numiter(resol-64,5) = length(Close);
end

% Weighted A* with e=5
for resol = 65:67
    % Initialize and create open and close spaces
    Open = zeros(0);  Open = cat(1,Open,start(resol-64)); Close = zeros(0);
    V = zeros(length(tot_vert(resol-64))); e = 5;
    for ini = 1:tot_vert(resol-64)
        if ini ~= start(resol-64)
            V(ini) = inf;
        end
    end
    
    while ~any(Close(:) == goal(resol - 64))
        Vtemp = 0;
        for f = 1:length(Open)
            Vtest = V(Open(f));
            h = norm(coords.(char(resol))(Open(f),:)-coords.(char(resol))(goal(resol-64),:));
            Vtemp(f) = Vtest + e*h;
        end
        % remove vertex with the lowest cost from open to close
        [val,node] = min(Vtemp);
        xj= Open(node);
        Open = Open(~ismember(Open,xj)); Close = cat(1,Close,xj);
        % find the neighbor
        index = find(data.(char(resol)).from == xj); Vnew = V; neighbor = 0;
        for temp = 1:length(index)
            neighbor(temp) = data.(char(resol)).to(index(temp));
            Vnew(neighbor(temp)) = data.(char(resol)).cost(index(temp)) + V(xj);
        end
        neighbor = neighbor(~ismember(neighbor,Close));
        for xi = 1:length(neighbor)
            if find(Open(:) == neighbor(xi))
                if Vnew(neighbor(xi)) < V(neighbor(xi))
                    V(neighbor(xi)) = Vnew(neighbor(xi));
                end
            else
                Open = cat(1,Open,neighbor(xi));
                V(neighbor(xi)) = Vnew(neighbor(xi));
            end
        end
        
    end
    output_cost(resol-64,6) = V(goal(resol - 64));
    output_numiter(resol-64,6) = length(Close);
end

% create output file
fid = fopen('output_costs.txt','w');
fprintf(fid,'%.6f ',output_cost(1,:));
fprintf(fid,'\n');
fprintf(fid,'%.6f ',output_cost(2,:));
fprintf(fid,'\n');
fprintf(fid,'%.6f ',output_cost(3,:));
fclose(fid);

% create output file
fid = fopen('output_numiter.txt','w');
fprintf(fid,'%d ',output_numiter(1,:));
fprintf(fid,'\n');
fprintf(fid,'%d ',output_numiter(2,:));
fprintf(fid,'\n');
fprintf(fid,'%d ',output_numiter(3,:));
fclose(fid);