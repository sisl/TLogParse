
using BayesNets
using PGFPlots
using DataFrames
using DataStructures.Stack
using LightGraphs

function toGPH(b,filename)
    #b - BayesNet that needs to be saved to a file
    #filename - String of the csv file used to load the data
    #
    #Saves a BayesNet to a .gph file
    
    outname = filename[1:end-3] * "gph"
    f = open(outname,"w")
    bnames=b.names
    for name in bnames
        for parent in parents(b,name)
            write(f,string(parent,",",name,"\n"))
        end
    end
    close(f)
end

function importGPH(filenme)
    #filename - String of the csv file used to load the data
    #
    #Loads a BayesNet from a .gph file
    inName = filenme[1:end-3] * "gph"
    b=BayesNet(headerSym)
    f=open(inName)
    line = readline(f)
    while !isempty(line)
        text=split(line,r"\"|,|\n",false)
        addEdge!(b,symbol(text[1]),symbol(text[2]))
        line = readline(f)
    end
    close(f)
    b
end

function sameGraph(b1,b2)
    #b1 - BayesNet 1
    #b2 - BayesNet 2
    #
    #Compares two BayesNets and returns true if they are identical
    nodes = b1.names
    flag = true
    for i=1:length(nodes)
        parents1 = parents(b1,nodes[i])
        parents2 = parents(b2,nodes[i])
        if length(parents1)!=length(parents2)
            flag = false
        else
            for j=1:length(parents1)
                if parents1[j] != parents2[j]
                    flag = false
                end
            end
        end
    end
    flag
end
    

function getNodeScores(alpha,N)
    #alpha - vector of alpha values
    #N     - vector of counts
    #
    #Returns a vector of the individual logBayesScore of each node in a network
    n=length(N)
    p = zeros(n)
    for i = 1:n
        if !isempty(N[i])
            p[i] += sum(lgamma(alpha[i] + N[i]))
            p[i] -= sum(lgamma(alpha[i]))
            p[i] += sum(lgamma(sum(alpha[i],1)))
            p[i] -= sum(lgamma(sum(alpha[i],1) + sum(N[i],1)))
        end
    end
    p
end

#function editAlpha!(alpha,i,del)
#    if del
#        alpha[i]=ones(2,convert(Int64,length(alpha[i])/4))
#    else
#        alpha[i]=ones(2,convert(Int64,length(alpha[i])))
#    end
#    alpha
#end

function randomStart(headerSym)
    #headerSym - Symbolic array names of nodes
    #
    #Creates a Bayesian Network with random edges.
    #The network will be acyclic
    
    b = BayesNet(headerSym[randperm(length(headerSym))])
    nodes = b.names
    r = rand(length(nodes),length(nodes))*3
    for i = 1:length(nodes)
        for j = 1:i-1
            if r[i,j]<0.1
                addEdge!(b,nodes[i],nodes[j])
                if cyclic(b,i)
                    removeEdge!(b,nodes[i],nodes[j])
                end
            end
            if r[i,j] > 2.9
                addEdge!(b,nodes[j],nodes[i])
                if cyclic(b,i)
                    removeEdge!(b,nodes[j],nodes[i])
                end
            end
        end
    end
    b;
end

function cyclic(b,i)
    #b - BayesNet
    #i - index of node that needs to be checked
    #
    #Checks if a node in a BayesNet is part of a cycle
    #Performs a depth first search of the parents of the given node
    nodes = b.names
    flag = false
        visited = Set({nodes[i]})
        current = nodes[i]
        ps = parents(b,current)
        parentStack = Stack(NodeName)
    
        for j=ps
            push!(parentStack,j)
        end
        while length(parentStack) > 0
        current = pop!(parentStack)
            if isempty(intersect(visited,Set({current})))
                visited = union(visited,Set({current}))
                ps = parents(b,current)
                for j=ps
                    push!(parentStack,j)
                end
            elseif !isempty(intersect(Set({nodes[i]}),Set({current})))
                flag = true
            end
        end
    flag  
end

#function cyclic(b,node)
#    flag = false
#    visited = Set({node})
#    current = node
#        ps = parents(b,current)
#        parentStack = Stack(NodeName)
#        for j=ps
#            push!(parentStack,j)
#        end
#        while length(parentStack) > 0
#        current = pop!(parentStack)
#            if isempty(intersect(visited,Set({current})))
#                visited = union(visited,Set({current}))
#                ps = parents(b,current)
#                for j=ps
#                    push!(parentStack,j)
#                end
#            elseif !isempty(intersect(Set({node}),Set({current})))
#                flag = true
#            end
#        end
#    flag  
#end

function modifyN(b,d,r,m,index)
    #b     - BayesNet
    #d     - 2D array of data
    #r     - Array of number of different values each nodes in b can take
    #m     - Int64 number of nodes
    #index - Int64 number of node
    #
    #Recalculates N for a given node
    p=int(collect(in_neighbors(b.dag,index)));
    temp=zeros(2,2^length(p));
    for di = 1:m
        k = d[index,di]
        j = 1
        if !isempty(p)
            ndims = length(r[p])
            j = int(d[p,di][1])
            stride = 1
            for kk=2:ndims
                stride = stride * r[p][kk-1];
                j += (int(d[p,di][kk])-1) * stride;
            end
        end
        temp[k,j] += 1.;
    end
    temp;
end

function compareGraph(oldScore,array)
    #oldscore - double old node score before change was made
    #array    - new N array for BayesNet with change made
    #
    #Calculates the score of the node with the change made (edge added/removed/reversed)
    #Returns the difference in scores. Positive p means the score increased.
    p = 0.
    sumAlphaN = ones(convert(Int64,length(array)/2))'*2
    alphaN = ones(size(array))
    p += sum(lgamma(alphaN + array))
    p -= sum(lgamma(alphaN))
    p += sum(lgamma(sumAlphaN))
    p -= sum(lgamma(sumAlphaN + sum(array,1)))
    p -= oldScore

    p
end

function getText(b,changeAdd,changeVector)
    #b            - BayesNet
    #changeAdd    - double value describing if the edge was added, reversed, or removed
    #changeVector - array of node indices that were changed
    if changeAdd>0.5
        text = "Added Edge: " * string(b.names[changeVector[1]]) * "->" * string(b.names[changeVector[2]])
    elseif changeAdd >-0.5 && changeAdd < 0.5
        text = "Reversed Edge: " * string(b.names[changeVector[1]]) * "->" * string(b.names[changeVector[2]])
    elseif changeAdd > -1.5 && changeAdd < -0.5
        text = "Removed Edge: " * string(b.names[changeVector[1]]) * "->" * string(b.names[changeVector[2]])
    end
end

function randomize(bestGraph)
    #bestGraph - BayesNet
    #
    #Randomly reverse edges in a BayesNet
    #This serves to reverse multiple edges at once in hopes of overcoming local maxima
    b = deepcopy(bestGraph)
    for i = 1:length(b.names)
        for j = 1:length(b.names)
            if hasEdge(b,b.names[i],b.names[j])
                if rand() < 0.15
                    removeEdge!(b,b.names[i],b.names[j])
                    addEdge!(b,b.names[j],b.names[i])
                    if !cyclic(b,i)
                        println(string("reversed: ",b.names[i],"=>",b.names[j]))
                    else
                        removeEdge!(b,b.names[j],b.names[i])
                        addEdge!(b,b.names[i],b.names[j])
                    end
                end
            end
        end
    end
    b
end

function removeWorstEdges(bestGraph,num)
    #bestGraph - BayesNet
    #num       - Int64 number of edges that should be removed
    #
    #Searches through bestGraph and removes the least helpful edges in a Bayesian Network
    b=deepcopy(bestGraph)
    d=indexData(b,df)
    alpha=prior(b)
    N=statistics(b,df)
    r = [length(domain(b, node).elements) for node in b.names]
    (n,m)=size(d)
    oldScores = getNodeScores(alpha,N)
    dict = Dict{(Symbol,Symbol),Float64}()
    for i=1:length(b.names)
        for parent in parents(b,b.names[i])
            removeEdge!(b,parent,b.names[i])
            score = compareGraph(oldScores[i],modifyN(b,d,r,m,i))
            if score<0
                dict[(parent,b.names[i])] = score 
            end
            addEdge!(b,parent,b.names[i])
        end
    end
    vals=sort(collect(values(dict)))
    boundary = vals[length(vals)-num+1]-0.0000001
    for key in collect(keys(dict))
        if dict[key]>=boundary
            println("Reversed: ",key[1],"=>",key[2],"  ",dict[key])
            removeEdge!(b,key[1],key[2])
            addEdge!(b,key[2],key[1])
        end
    end
    b
end

function computeBestNeighbor!(b,df)
    #b  - BayesNet
    #df - data read from csv file
    #
    #This function computes the best neighbor of Bayesian Network, b.
    #A neighbor is defined as a network that is achieved by adding, reversing, or removing an edge in the network
    #This function is used in a random restart hill climbing algorithm to try to find the global optimum Bayes Net
    d=indexData(b,df)
    alpha=prior(b)
    N=statistics(b,df)
    r = [length(domain(b, node).elements) for node in b.names]
    (n,m)=size(d)
    nodes = b.names
    oldScores = getNodeScores(alpha,N)
    currentScore = 0.
    changeAdd = -2
    changeVector = [0 0]
    
    #Iterate through pairs of nodes
for i=1:length(nodes)
        for j=1:(i-1)
            if i!=j
                
                #Three cases to check: There exists an edge i->j, j->i, or no edge
                if hasEdge(b,nodes[i],nodes[j])
                    removeEdge!(b,nodes[i],nodes[j])  
                    score = compareGraph(oldScores[j],modifyN(b,d,r,m,j))
                    if currentScore < score
                        changeAdd = -1
                        changeVector = [i j]
                        currentScore = score
                    end
                    addEdge!(b,nodes[j],nodes[i])
                    if !cyclic(b,i)
                        score += compareGraph(oldScores[i],modifyN(b,d,r,m,i))
                        if currentScore < score
                            changeAdd = 0
                            changeVector = [i j]
                            currentScore = score
                        end
                    end
                    removeEdge!(b,nodes[j],nodes[i])
                    addEdge!(b,nodes[i],nodes[j])
                elseif hasEdge(b,nodes[j],nodes[i])
                    removeEdge!(b,nodes[j],nodes[i])
                    score = compareGraph(oldScores[i],modifyN(b,d,r,m,i))
                    if currentScore < score
                        changeAdd = -1
                        changeVector = [j i]
                        currentScore = score
                    end
                    addEdge!(b,nodes[i],nodes[j])
                    if !cyclic(b,i)
                        score += compareGraph(oldScores[j],modifyN(b,d,r,m,j))
                        if currentScore < score
                            changeAdd = 0
                            changeVector = [j i]
                            currentScore = score
                        end
                    end
                    removeEdge!(b,nodes[i],nodes[j])
                    addEdge!(b,nodes[j],nodes[i])
                elseif !hasEdge(b,nodes[j],nodes[i])
                    addEdge!(b,nodes[i],nodes[j])
                    if !cyclic(b,i)
                    score = compareGraph(oldScores[j],modifyN(b,d,r,m,j))
                        if currentScore < score
                            changeAdd = 1
                            changeVector = [i j]
                            currentScore = score
                        end
                    end
                    removeEdge!(b,nodes[i],nodes[j])
                    addEdge!(b,nodes[j],nodes[i])
                    if !cyclic(b,i)
                        score = compareGraph(oldScores[i],modifyN(b,d,r,m,i))
                        if currentScore < score
                            changeAdd = 1
                            changeVector = [j i]
                            currentScore = score
                        end
                    end
                    removeEdge!(b,nodes[j],nodes[i])
                end
            end
        end
    end
    
    #Modify b to be the best neighbor
    if changeAdd > 0.5
        addEdge!(b,nodes[changeVector[1]],nodes[changeVector[2]])
    elseif changeAdd < -0.5 && changeAdd > -1.5
        removeEdge!(b,nodes[changeVector[1]],nodes[changeVector[2]])
    elseif changeAdd>-0.5 && changeAdd<0.5
        removeEdge!(b,nodes[changeVector[1]],nodes[changeVector[2]])
        addEdge!(b,nodes[changeVector[2]],nodes[changeVector[1]])
    end
    (b,getText(b,changeAdd,changeVector))
end

#Read in data
filename = "../data/medium.csv"
f = open(filename)
headerRaw= readline(f)
close(f)
headerText = split(headerRaw,r"\"|,|\n",false)
headerSym = symbol("")
for i = 1:length(headerText)
    headerSym = [headerSym symbol(headerText[i])]
end
headerSym = headerSym[2:end]
df = readtable(filename);

#Load an existing graph or make a new graph
#bestGraph = importGPH("large.gph")
bestGraph=BayesNet(headerSym)
bestScore = logBayesScore(bestGraph,df)
println(bestScore)
bestscores = []
currentscores = [];

numrestarts=1
for i = 1:numrestarts
    println("Round ",i)
    #a = time()
    #b=removeWorstEdges(bestGraph,7)
    b=randomStart(headerSym)
    #b=importGPH("large_temp.gph")
    #b=importGPH("large_temp1.gph")
    currentScore=logBayesScore(b,df)
    println(currentScore)
    (b,text)=computeBestNeighbor!(b,df);
    newScore = logBayesScore(b,df)
    #toGPH(b,"large_temp.csv")
    while abs(newScore-currentScore)>0.00000001
        println(string(newScore) * ", " * text)
        currentScore=newScore
        (b,text)=computeBestNeighbor!(b,df);
        newScore = logBayesScore(b,df);
        toGPH(b,"large_temp.csv")
    end
    println(string("FinalScore: ",newScore))
    #toGPH(b,string("large-",i+10,".gph"))
    if currentScore > bestScore
        bestScore = currentScore
        bestGraph = deepcopy(b);
        toGPH(bestGraph,filename)
    end
    if isempty(bestscores)
        bestscores = bestScore
        currentscores = currentScore
    else
        bestscores = [bestscores, bestScore]
        currentscores = [currentscores, currentScore]
    end
    println("END ROUND ",i)
end

#b=importGPH("large.gph")
#b2=importGPH("large-4.gph")
#logBayesScore(b2,df)
#b3=BayesNet(headerSym);
#for i = 1:length(b.names)
#    for j=1:length(b.names)
#        if hasEdge(b,b.names[i],b.names[j]) && hasEdge(b2,b.names[i],b.names[j])
#            addEdge!(b3,b.names[i],b.names[j])
#        end
#    end
#end
#toGPH(b3,"Large_Intersection.jpg")
#logBayesScore(b3,df)

#Axis([Plots.Linear(1:length(bestscores),bestscores,mark="none",legendentry="Best Score"),
#    Plots.Linear(1:length(currentscores),currentscores,mark="none",legendentry="Score")
#    ],legendPos="south east")
