 
costpmat = [];
costcmat = [];
costfornode = 1;
costforedge = 1;
costforcommsnode = 1

nodevalues = transpose([1,1,1,1,1,1,1,1,1,1,1,1,1,1])
attackcostmat = [];
attackoptioncosts = {}

GPOWER = [1	1	costfornode	;
2	2	costfornode	;
3	3	costfornode	;
4	4	costfornode	;
5	5	costfornode	;
6	6	costfornode	;
7	7	costfornode	;
8	8	costfornode	;
9	9	costfornode	;
10	10	costfornode	;
11	11	costfornode	;
12	12	costfornode	;
13	13	costfornode	;
14	14	costfornode	;
1	2	costforedge	;
1	5	costforedge	;
2	3	costforedge	;
2	4	costforedge	;
2	5	costforedge	;
3	4	costforedge	;
4	5	costforedge	;
4	7	costforedge	;
4	9	costforedge	;
5	6	costforedge	;
6	11	costforedge	;
6	12	costforedge	;
6	13	costforedge	;
7	8	costforedge	;
7	9	costforedge	;
9	10	costforedge	;
9	14	costforedge	;
10	11	costforedge	;
12	13	costforedge	;
13	14	costforedge	;];

GComms= [1	2	costforcommsnode	;
         1	5	costforcommsnode	;
         1	3	costforcommsnode	;
         1	4   costforcommsnode	;
         1	7	costforcommsnode	;
         1	9	costforcommsnode	;
         1	6	costforcommsnode	;
         1	11	costforcommsnode	;
         1	12	costforcommsnode	;
         1	13	costforcommsnode	;
         1	8	costforcommsnode	;
         1	10	costforcommsnode	;
         1	14	costforcommsnode	;
         1	15	costforcommsnode	;];



gp = graph(GPOWER(:,1),GPOWER(:,2),GPOWER(:,3));

gc = graph(GComms(:,1),GComms(:,2),GComms(:,3));



gp.Nodes.Name = {'A', 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'N' 'M' 'O'}';

gc.Nodes.Name = {'SO','A', 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'N' 'M' 'O'}';


nodesleftname = transpose(['A','B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'N' 'M' 'O']);

mincostpath = [];
avcostmat = [];
costofminpointattackmat = [];
pathofminpointattackmat = [];
meterallocneededmat = [];



remainingphyscostmat = []; 
remainingcommscostmat = []; 
totalcostmat = [];
sgcell = {}
 

totalcostcell={};
totalcostcellsearch={};

comboscell = {};
sgCcell={};

sgPcell={};
sgPbuscell={};

graphmat = [];

nodalpathing = [];
strategynodecost = [];

for initialrun = 1:1

%weighting every cost to attack buses%

for busnumber = 1:size(nodesleftname,1)

    %building initial subgraph from weighted graph%
    
nodename = nodesleftname(busnumber);
N=[(neighbors(gp,nodename));nodename];
N = unique(N)
sgcell{busnumber} = subgraph(gp,N);

for physicalsubgraph = 1:1


%edges to remove (not connected to target node)%

nodestoremove = sgcell{busnumber}.Edges.EndNodes

nodestoremovepruned1 = find(contains(nodestoremove(:,1),nodename))
nodestoremovepruned2 = find(contains(nodestoremove(:,2),nodename))
nodestoremovepruned3 = unique([nodestoremovepruned1;nodestoremovepruned2])

nodestoremovemat = []  

for j = 1:size(nodestoremove,1)

    if    contains(nodestoremove{size(nodestoremove,1)+j}(1,1),nodename)==1
    a=1
    elseif contains(nodestoremove{j}(1,1),nodename)==1
    a = 1    
    elseif contains(nodestoremove{j}(1,1),nodestoremove{size(nodestoremove,1)+j}(1,1))==1
    a = 1    
    else      
    a = 0   
    end
nodestoremovemat = [nodestoremovemat;a] 

end

prunededges = find(nodestoremovemat==0);

sgcell{busnumber} = rmedge(sgcell{busnumber},prunededges);

costtotakephys = sum(sgcell{busnumber}.Edges.Weight);

costpmat = [costpmat; sum(sgcell{busnumber}.Edges.Weight)];


end

totalcostmat = [sum(sgcell{busnumber}.Edges.Weight),0,0];

for mixedsubgraphs = 1:1
    
    
for comboslength = 1:size(N,1)
    
    combos= nchoosek(N,comboslength)
    
    
    %store for later reference%
    comboscell{busnumber}(comboslength) = {combos}
    
for i = 1:size(combos,1)
    
sgpruned = sgcell{busnumber};
        
sgpruned = rmedge(sgpruned, combos(i,:),combos(i,:));
 
remainingphyscost = sum(sgpruned.Edges.Weight);

%next remove attached branches% %identify node neibors%

for j= 1:size(combos(i,:),2)
       
     N2 = neighbors(sgpruned,combos{i,j});
          
for h = 1:size(N2,1)
    
         
         %remove those nodes based on connection to target%
         
         
    sgpruned = rmedge(sgpruned, combos{i,j},N2(h));
    
    
end


end

C1 = combos(i,:);
C1{end+1} = 'SO';
sgc = subgraph(gc,C1);

%storing subgraphs for later reference%

sgCcell{i,j}=sgc;
sgPcell{i,j}=sgpruned;

sgCbuscell{busnumber}=sgCcell;

sgPbuscell{busnumber}=sgPcell;


%weighing communications subgraphs%
 
commscost = sum(sgc.Edges.Weight);
physcost = sum(sgpruned.Edges.Weight); 
 
totalcostmat = [totalcostmat;commscost+sum(sgpruned.Edges.Weight),i,j];

%also store subgraphs%

end

totalcostcell{busnumber}= totalcostmat

end

end

sgCcell = [];
sgPcell = [];

end

%Min node identification and pruning (or replace with next bus selection)%

for minident = 1:1

for mincellident = 1:1

mincellmat = [];


for num = 1:length(nodesleftname)

    
min(totalcostcell{num}(:,1))

mincellmat = [mincellmat;min(totalcostcell{num}(:,1))
];

end


%min bus target%

[minbusvalue,minbusindex]=min(mincellmat);

%min combination of comms/phys%

[mincombvalue,mincombindex]=min(totalcostcell{minbusindex}(:,1));


end

end


end

for firstnodecost = 1:1
%nodal pathing%

nodalpathing = [nodalpathing;nodesleftname(minbusindex)];


%strategy cost%

Ccost1 = (sgCbuscell{minbusindex}(totalcostcell{minbusindex}(mincombindex,2),totalcostcell{minbusindex}(mincombindex,3)));
Pcost1 = (sgPbuscell{minbusindex}(totalcostcell{minbusindex}(mincombindex,2),totalcostcell{minbusindex}(mincombindex,3)));

Ccost = sum(Ccost1{1,1}.Edges.Weight);
Pcost = sum(Pcost1{1,1}.Edges.Weight);

strategynodecost = [strategynodecost;Ccost+Pcost;]

end


%now have the sub graphs stored need to prune the edges from these sub
    
    


