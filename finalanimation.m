close all;
clear all;

agents = 10;


list0 = [ 'A', 'C', 'D', 'E', 'K', 'E', 'K', 'O']
list0_num = [1,3,4,5,11,5,11,15]
%edge0 = ['AC', 'CD','DE','EK','KE','EK','KO']

list1 = ['B', 'C', 'D', 'E', 'F', 'E', 'K', 'O']
list1_num = [2,3,4,5,6,5,11,15]

%edge1 = ['BC', 'CD','DE','EF','FE','EK','KO']

list2 = [ 'I', 'H', 'J', 'H', 'G', 'F', 'E', 'K']
list2_num = [9,8,10,8,7,6,5,11]
%edge2 = ['IH', 'HJ','JH','HG','GF','FE','EK']

list3 = ['J', 'H', 'G', 'F', 'E', 'K', 'O']
list3_num = [10,8,7,6,5,11,15]

list4 = ['M', 'L', 'N', 'L', 'K', 'E', 'K','O']
list4_num = [13,12,14,12,11,5,11,15]

list5 = ['N', 'L', 'K', 'O', 'P', 'O', 'K', 'O']
list5_num = [14,12,11,15,16,15,11,15]

list6 = ['R', 'Q', 'P', 'S', 'P', 'O', 'K', 'O']
list6_num = [18,17,16,19,16,15,11,15]

list7 = ['T', 'S', 'U', 'V', 'U', 'S', 'P', 'O']
list7_num = [20,19,21,22,21,19,16,15]

list8 = ['W', 'V', 'X', 'V', 'U', 'S', 'P', 'O']
list8_num = [23,22,24,22,21,19,16,15]


list9 = ['X', 'V', 'U', 'S', 'P', 'O', 'K']
list9_num = [24,22,21,19,16,15,11]

s = [1,2,3,4,5,5,6,7,8,8,11,11,12,12,15,16,16,17,19,19,21,22,22];
t =  [3,3,4,5,6,11,7,8,9,10,12,15,13,14,16,17,19,18,20,21,22,23,24];
weights = [10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10];
names = {'A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X'}
robo_spawn = ['A','B','I','J','M','N','R','T','W','X']
labels = ["AC","BC","CD","DE","EF","EK","FG","GH","HI","HJ","KL","KO","LM","LN","OP","PQ","PS","QR","ST","SU","UV","VW","VX"]; 
G = digraph(s,t,weights,names);
G = flipedge(G);
G.Edges.Label = labels';
p = plot(G)
p.ShowArrows = 0
p.Marker = '+'
p.EdgeColor = [0.5 0.5 0.5]
X = p.XData
Y = p.YData
Z = p.ZData


path_0_x = X(list0_num)
path_0_y = Y(list0_num)
path_0_z = 1;

path_1_x = X(list1_num)
path_1_y = Y(list1_num)
path_1_z = 2;

path_2_x = X(list2_num)
path_2_y = Y(list2_num)
path_2_z = 3;

path_3_x = X(list3_num)
path_3_y = Y(list3_num)
path_3_z = 4;

path_4_x = X(list4_num)
path_4_y = Y(list4_num)
path_4_z = 5;

path_5_x = X(list5_num)
path_5_y = Y(list5_num)
path_5_z = 6;

path_6_x = X(list6_num)
path_6_y = Y(list6_num)
path_6_z = 7;

path_7_x = X(list7_num)
path_7_y = Y(list7_num)
path_7_z = 8;

path_8_x = X(list8_num)
path_8_y = Y(list8_num)
path_8_z = 9;


path_9_x = X(list9_num)
path_9_y = Y(list9_num)
path_9_z = 10;



curve1 = animatedline('Color','b','LineWidth',1);
curve2 = animatedline('Color','r','LineWidth',1);
curve3 = animatedline('Color','g','LineWidth',1);
curve4 = animatedline('Color','k','LineWidth',1);

curve5 = animatedline('Color','#EDB120','LineWidth',1);
curve6 = animatedline('Color','m','LineWidth',1);
curve7 = animatedline('Color','#D95319','LineWidth',1);
curve8 = animatedline('Color','c','LineWidth',1);
curve9 = animatedline('Color','#77AC30','LineWidth',1);
curve10 = animatedline('Color','#7E2F8E','LineWidth',1);



set(gca,'XLim',[0 6.75],'YLim',[0 13],'ZLim',[0 15]);

view(3);

grid on
hold on
custom5 = [0.9290 0.6940 0.1250];
custom7 = [0.8500 0.3250 0.0980];
custom9 = [0.4660 ,0.6740, 0.1880];
custom10 = [0.4940 0.1840 0.5560];
size=40;        % size of marker
head1 = scatter3(gca,nan,nan,nan,size,'filled','b');
head2 = scatter3(gca,nan,nan,nan,size,'filled','r');
head3 = scatter3(gca,nan,nan,nan,size,'filled','g');
head4 = scatter3(gca,nan,nan,nan,size,'filled','k');
head5 = scatter3(gca,nan,nan,nan,size,custom5,'filled');
head6 = scatter3(gca,nan,nan,nan,size,'filled','m');
head7 = scatter3(gca,nan,nan,nan,size,custom7,'filled');
head8 = scatter3(gca,nan,nan,nan,size,'filled','c');
head9 = scatter3(gca,nan,nan,nan,size,custom9,'filled');
head10 = scatter3(gca,nan,nan,nan,size,custom10,'filled');

lgd = legend;
%lgd.Layout.Tile = 1;




i = 1;

while 1
    if length(path_0_x) >= i
        addpoints(curve1,path_0_x(1,i),path_0_y(1,i),path_0_z(1));
        
        set(head1, 'xdata', path_0_x(1,i),'ydata',path_0_y(1,i),'zdata',path_0_z(1,1));
    end
    if length(path_1_x) >= i
        addpoints(curve2,path_1_x(1,i),path_1_y(1,i),path_1_z(1));
        set(head2, 'xdata', path_1_x(1,i),'ydata',path_1_y(1,i),'zdata', path_1_z(1,1));
    end
    if length(path_2_x) >= i
        addpoints(curve3,path_2_x(1,i),path_2_y(1,i),path_2_z(1));
        set(head3, 'xdata', path_2_x(1,i),'ydata', path_2_y(1,i),'zdata', path_2_z(1,1));
    end
    if length(path_3_x) >= i
        addpoints(curve4,path_3_x(1,i),path_3_y(1,i),path_3_z(1));
        set(head4, 'xdata', path_3_x(1,i),'ydata', path_3_y(1,i),'zdata', path_3_z(1,1));
    end
    if length(path_9_x) >= i
        addpoints(curve10,path_9_x(1,i),path_9_y(1,i),path_9_z(1));
        set(head10, 'xdata', path_9_x(1,i),'ydata', path_9_y(1,i),'zdata', path_9_z(1,1));
    end
    if length(path_4_x) >= i
        addpoints(curve5,path_4_x(1,i),path_4_y(1,i),path_4_z(1));
        set(head5, 'xdata', path_4_x(1,i),'ydata', path_4_y(1,i),'zdata', path_4_z(1,1));
    end
    if length(path_5_x) >= i
        addpoints(curve6,path_5_x(1,i),path_5_y(1,i),path_5_z(1));
        set(head6, 'xdata', path_5_x(1,i),'ydata', path_5_y(1,i),'zdata', path_5_z(1,1));
    end
    if length(path_6_x) >= i
        addpoints(curve7,path_6_x(1,i),path_6_y(1,i),path_6_z(1));
        set(head7, 'xdata', path_6_x(1,i),'ydata', path_6_y(1,i),'zdata', path_6_z(1,1));
    end
    if length(path_7_x) >= i
        addpoints(curve8,path_7_x(1,i),path_7_y(1,i),path_7_z(1));
        set(head8, 'xdata', path_7_x(1,i),'ydata', path_7_y(1,i),'zdata', path_7_z(1,1));
    end
    if length(path_8_x) >= i
        addpoints(curve9,path_8_x(1,i),path_8_y(1,i),path_8_z(1));
        set(head9, 'xdata', path_8_x(1,i),'ydata', path_8_y(1,i),'zdata', path_8_z(1,1));
    end
    %addpoints(curve3,y(i,7),y(i,8),y(i,9));
    %addpoints(curve4,y(i,10),y(i,11),y(i,12));
        
    drawnow
    pause(5);
    i = i + 1;
end

