%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your job is to solve two different types of swarming problems however you
%% see fit. In the code below, I have left some ???? which is where your code 
%% will have to go.
%%
%% The way you run the code is by running matlab from the directory
%% in which you stored the four files
%% hw3.m
%% load_network.m
%% plotsol.m
%% disk.m
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The two different problems are: 
%% (i) Make sure that the swarm stays connected during rendezvous
%% (ii) Make sure that the robots don't run into each other
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;

figure
%% Proximity disk
Delta=0.5; % interaction distance

%% You should try your solutions on three different, initial networks that
%% you load using the following function call

network_no=2; %% or 2 or 3
[X,n,N]=load_network(network_no,Delta);
% n = dimension of each robot (n>1)
% N = total number of robots
% X = n x N vector containing the initial robot positions


%% Some numerical integration parameters
dt=0.003; % numerical steplength
Tf=2; % final time

t=0; 
iter=1;

dist = .2;
p = 3;
b = -tan(-pi/2 + pi/p);
%weightfcn = @(d) tan(-pi/2 + pi * d/Delta) + b;
 weightfcn = @(d) (.5 .* (2 .* Delta - d)./(Delta - d).^2) * (d - dist);
dist = @(xi, xj) sqrt( (xi - xj)' * (xi - xj));
weight = @(xi, xj) weightfcn(dist(xi, xj)); 

% H for hysterisys
epsilon = .05;
H = zeros(size(disk(X,N,Delta)));
dmin = inf;

while (t<=Tf);

%% A is the adjacency matrix associated with the system
%% using a disk graph. 
  A=disk(X,N,Delta);

  DX=zeros(n,N); %% Here is where we store the derivatives
  for i=1:N;
    for j=1:N;
      if (A(i,j)==1);

%% ?????????????????????????????????
dmin = min(dist(X(:,j),X(:,i)), dmin);
if H(i, j) == 0     % this is a new edge
    if dist(X(:,j),X(:,i)) < Delta-epsilon  % don't head towards
        H(i, j) = 1;
    end
end
w = weight(X(:,j),X(:,i)) * H(i, j);
DX(:,i)=DX(:,i)+ w .* (X(:,j)-X(:,i));   % The consensus equation
%% ?????????????????????????????????
%% Note, here I've given the consensus equation.
%% Your job is to change this in order to make the robots
%% satisfy the three (not all at once, mind you) questions
%% that I want you to think about...

  end; end; end;

%% Update the states using an Euler approximation
  for i=1:N;
    X(:,i)=X(:,i)+dt.*DX(:,i);
  end;

%% Update time
  t=t+dt;


%% Plot the solution every 10 iterations
  if (mod(iter,10)==0);
    plotsol(X,N,A,Delta);
  end;

  iter=iter+1;
end;
 
dmin


