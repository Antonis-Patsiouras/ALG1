% Αλγόριθμος Alg-1
% Καθορισμός global μεταβλητών
global d space robot start goal objdir poses;
% Η διάσταση του χώρου
d=30;
poses=zeros(1,2);
%καθορισμός του χώρου
space=zeros(d,d);
% καθορισμός αφετηρίας και τερματισμού
start=[30 10];
goal=[1 20];
% Υπολογισμός των σημείων που ανήκουν στην m-line
mpoints=max(abs(goal(1)-start(1)),abs(goal(2)-start(2)));
mr=round(linspace(start(1),goal(1),mpoints+1));
mc=round(linspace(start(2),goal(2),mpoints+1));
% Μαρκάρισμα των κελιών του πλέγματος που ανήκουν στην m-line
% με τον αριθμό 3
space(sub2ind(size(space), mr, mc))=3;
% Μαρκάρισμα της αφετηρίας και του τερματισμού
% με τους αριθμούς 1 και 4 αντίστοιχα
space(start(1),start(2))=1;
space(goal(1),goal(2))=4;
% αρχική θέση του ρομπότ
robot=start;
% Καθορισμός των κελιών που καταλαμβάνονται από εμπόδια
ObstRows = [20 20 20 20 20 20 20 20 20 20 20 19 19 19 19 19 19 19 19 19 19 19 18 18 18 18 18 18 18 18 18 18 18 21 22 21 21 22 23];
ObstCols = [10 11 12 13 14 15 16 17 18 19 20 10 11 12 13 14 15 16 17 18 19 20 10 11 12 13 14 15 16 17 18 19 20 12 12 16 08 08 08];
ObstRows = [ObstRows 11 11 11 11 11 11 11 11 11 11 11 10 10 10 10 10 10 10 10 10 10 10 09 09 09 09 09 09 09 09 09 09 09 08 08 07 07];
ObstCols = [ObstCols 2 13 14 15 16 17 18 19 20 21 22 12 13 14 15 16 17 18 19 20 21 22 12 13 14 15 16 17 18 19 20 21 22 21 22 21 22];
% Μαρκάρισμα των εμποδίων με τον αριθμό 2
space(sub2ind(size(space), ObstRows, ObstCols))=2;
% Εμφάνιση του πλέγματος
fig=figure;
fig.WindowState = 'maximized';
axis([0,d,0,d]);
axis ij
grid on
hold on
% Σημαίες
GoalFound=false;
GoalSeek=true;
ObstAvoid=false;
titlos='Παρακολούθηση m-line';
% Η κατεύθυνση κίνησης γύρω από το εμπόδιο είναι ανύπαρκτη
objdir=0;
% εμφάνιση του πλέγματος
DispGrid(titlos);
% βρόχος επανάληψης
while ~GoalFound
    % Αν το ρομπότ είναι σε κατάσταση κίνησης προς τον τερματισμό
    if GoalSeek
        % υπολόγισε την νέα θέση ακολουθώντας την m-line
        newpos=followmline();
        % αν όμως είναι άδεια (εμπόδιο)
        if isempty(newpos) % εμπόδιο
            % καθόρισε οτι η κατάσταση είναι "Αποφυγή Εμποδίου"
            GoalSeek=false;
            ObstAvoid=true;
            titlos='Αποφυγή Εμποδίου';
        end
    end
    % Αν το ρομπότ είναι σε κατάσταση αποφυγής εμποδίου
    if ObstAvoid % αποφυγή εμποδίου
        % υπολόγισε την νέα θέση ακολουθώντας το περίγραμμα του εμποδίου
        newpos=circobst();
        % αν βρεθεί ξανά η m-line
        if space(newpos(1), newpos(2))==3 % m-line
            % τότε καθορισε οτι η κατάσταση είναι "Κίνηση προς Τερματισμό"
            GoalSeek=true;
            ObstAvoid=false;
            % Η κατεύθυνση κίνησης γύρω από το εμπόδιο είναι ανύπαρκτη
            objdir=0;
            titlos='Παρακολούθηση m-line';
        end
    end
    % Μετακίνησε το ρομπότ στη νέα θέση
    robot=newpos;
    % εμφάνισε το πλέγμα
    DispGrid(titlos);
    %καθυστέρηση
    pause(0.001);
    % Έλεγος για άφιξη στον τερματισμό
    if isequal(robot,goal)
        GoalFound=true;
        DispGrid('Βρέθηκε ο Προορισμός !');
    end
end
% *********************************************************
% Συνάρτηση Εμφάνισης του Πλέγματος
function DispGrid(titlos)
colorarr={'white' 'green' 'black' 'yellow' 'red'};
global space robot;
cla; % καθαρισμός των αντικειμένων γραφικών. Προλαμβάνει Memory Leak !
[r,c]=size(space);
for i=1:r
    for j=1:c
        color=colorarr{ space(i,j) + 1 };
        fill([j-1 j j j-1],[i i i-1 i-1],color);
    end
end
%display robot position
text(robot(2)-0.5,robot(1)-0.5,'R');
title(titlos);
end

% *********************************************************
% Συνάρτηση παρακολούθησης της γραμμής m-line
function newpos=followmline()
global d space robot goal direction;

% καθορισμός κατεύθυνσης της m-line
dr=goal(1)-robot(1);
dc=goal(2)-robot(2);
testpoints = zeros(1,3);
if (dr<0) % κατεύθυνση επάνω
    testpoints=[-1 0;-1 -1;-1 1];
    direction=2; % 2=επάνω (up)
elseif dr>0 % κατεύθυνση κάτω
    testpoints=[1 0;1 -1;1 1];
    direction=4; % 4=κάτω (down)
elseif dr==0
    if dc>0 % κατεύθυνση δεξιά
        testpoints=[0 1;-1 1;1 1];
        direction=1; % 1=δεξιά (right)
    else % κατεύθυνση αριστερά
        testpoints=[0 -1;-1 -1;1 -1];
        direction=3; % 3=αριστερά (left)
    end
end
% Αρχικά η νέα θέση του ρομπότ είναι κενή
newpos=[];

% Έλεγχος για παρακολούθηση της m-line
for i=1:3
    % δοκιμή ενός από τα 3 σημεία κατεύθυνσης
    tp=testpoints(i,:);
    % Αν είναι μέσα στον χώρο
    if all(robot+tp>0) && all(robot+tp<=d)
        % βρες το νέο σημείο
        newpoint=robot+tp;
        % και δοκίμασε αν δεν έχει εμπόδιο
        if space(newpoint(1),newpoint(2))>2
            newpos=newpoint;
            break;
        end
    end
end
end
% *********************************************************
% Συνάρτηση παρακολούθησης του περιγράμαμτος του εμποδίου
function newpos=circobst()
% objdir 1=right, 2=up, 3=left, 4=down
global space robot direction objdir poses;
poses(size(poses,1)+1,:) = robot;
% Αρχικοποίηση της κατεύθυνσης παράκαμψης
if objdir==0
    % αν η κατεύθυνση της m-line είναι "επάνω" (2)
    % τότε η κατέυθυνση της παράκαμψης είναι αριστερα (2+1=3)
    objdir=newdir(direction,+1);
end
% Έλεγχος για κίνηση στην κατεύθυνση objdir
success=false;
% Βρόχος επανάληψης
while ~success
    % καθρισμός κελιών δοκιμής
    switch objdir
        case 1 % right
            testpoints=[0 1;-1 1;1 1];
        case 2 % up
            testpoints=[-1 0;-1 -1;-1 1];
        case 3 % left
            testpoints=[0 -1;1 -1;-1 1];
        case 4 % down
            testpoints=[1 0;1 -1;1 1];
    end
    % εντοπίζεται το επόμενο κελί στην ίδια κατεύθυνση
    newpoint=robot+testpoints(1,:);
    % αν δεν έχει εμπόδιο
    
    if space(newpoint(1),newpoint(2))~=2
        if ismember(newpoint,poses,'rows') == 0
            % Το σχήμα είναι κυρτό
            % μετακινείται το ρομπότ στη νέα θέση
            newpos=newpoint;
            poses(size(poses,1)+1,:) = newpos;
        else
            objdir=newdir(objdir,+1);
            continue;
        end
        % ελέγχεται το εφαπτόμενο κελί
        newpoint=robot+testpoints(2,:);
        % αν δεν έχει εμπόδιο
        if space(newpoint(1),newpoint(2))~=2
            % το ρομπότ στρίβει στην επόμενη κατευθυνση CW
            objdir=newdir(objdir,+1); % αλλαγή κατεύθυνσης
        end
        success=true;
    else
        % το σχήμα είναι κοίλο
        % το ρομπότ στρίβει στην επόμενη κατεύθυνση CW
        objdir=newdir(objdir,-1);
    end
end
end
% *********************************************************
% Συνάρτηση υπολογισμού της νέας κατεύθυνσης
function dir=newdir(olddir,next)
if next>0
    dir=olddir+1; %change direction
    if dir>4
        dir=1;
    end
else
    dir=olddir-1;
    if dir<1
        dir=4;
    end
end
end